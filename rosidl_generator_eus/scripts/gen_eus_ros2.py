#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2026, JSK Lab, University of Tokyo.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of JSK Lab nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""Generate roseus (EusLisp) message classes for ROS 2 interfaces.

The ROS 1 geneus writes the ROS 1 wire format. On ROS 2 roseus passes
messages to rclcpp as serialized (CDR) buffers, so the generated classes
(de)serialize the XCDR1 little endian payload, which is what the ROS 2
rmw implementations (Fast DDS, Cyclone DDS, ...) exchange.  The 4 byte
encapsulation header is added / removed by roseus.so, so the buffers
here start at the CDR alignment origin.

The class API is the one of the ROS 1 geneus: pkg::Msg, :init with
keyword arguments, an accessor per field, pkg::SrvRequest /
pkg::SrvResponse and pkg::Srv for services.  On top of that

  builtin_interfaces/Time, Duration   -> ros::time, ros::duration
  uint8[], byte[], char[]             -> string
  integer arrays / float arrays       -> integer-vector / float-vector
  bool, string, message arrays        -> list

Usage:
  gen_eus_ros2.py [-o OUTPUT_DIR] [--source PKG:DIR] PKG [PKG ...]

writes OUTPUT_DIR/PKG/{msg,srv,action}/*.l.  The interface files are
read from the share directory of PKG in the ament index, or from DIR
(containing msg/ srv/ action/) when --source PKG:DIR is given.
"""

from __future__ import print_function

import argparse
import glob
import io
import json
import os
import shutil
import sys
import tempfile

from rosidl_adapter.parser import parse_action_string
from rosidl_adapter.parser import parse_message_string
from rosidl_adapter.parser import parse_service_string

TIME_TYPES = {
    ('builtin_interfaces', 'Time'): 'ros::time',
    ('builtin_interfaces', 'Duration'): 'ros::duration',
}

# (size in bytes, peek/poke type)
PRIMITIVES = {
    'bool': (1, ':char'),
    'byte': (1, ':char'),
    'char': (1, ':char'),
    'int8': (1, ':char'),
    'uint8': (1, ':char'),
    'int16': (2, ':short'),
    'uint16': (2, ':short'),
    'int32': (4, ':integer'),
    'uint32': (4, ':integer'),
    'int64': (8, ':long'),
    'uint64': (8, ':long'),
    'float32': (4, ':float'),
    'float64': (8, ':double'),
}
# types whose arrays are represented by a string
BYTE_TYPES = ('uint8', 'byte', 'char')
INTEGER_TYPES = ('byte', 'char', 'int8', 'uint8', 'int16', 'uint16',
                 'int32', 'uint32', 'int64', 'uint64')
FLOAT_TYPES = ('float32', 'float64')
STRING_TYPES = ('string', 'wstring')


def lisp_string(s):
    return '"%s"' % s.replace('\\', '\\\\').replace('"', '\\"')


class Field(object):
    """Field of a message, independent of the rosidl_adapter version."""

    def __init__(self, f, pkg):
        t = f.type
        self.name = f.name
        self.default = f.default_value
        self.is_array = t.is_array
        self.array_len = (t.array_size
                          if t.is_array and not t.is_upper_bound else None)
        if t.is_primitive_type():
            self.base = t.type
            self.msg = None
        else:
            msg_pkg = t.pkg_name or pkg
            self.base = None
            self.msg = (msg_pkg, t.type)
        self.type_str = str(t)

    @classmethod
    def make(cls, name, base=None, msg=None):
        self = cls.__new__(cls)
        self.name = name
        self.default = None
        self.is_array = False
        self.array_len = None
        self.base = base
        self.msg = msg
        self.type_str = base or '%s/%s' % msg
        return self

    @property
    def is_primitive(self):
        return self.base is not None

    @property
    def is_time(self):
        return self.msg in TIME_TYPES

    def eus_class(self, names):
        if self.is_time:
            return TIME_TYPES[self.msg]
        return names.eus_class(*self.msg)


class Names(object):
    """Mapping between ROS 2 type names and roseus class names."""

    def __init__(self):
        # (pkg, ros2 type name) -> eus class name (without package)
        self.alias = {}

    def eus_class(self, pkg, name):
        return '%s::%s' % (pkg, self.alias.get((pkg, name), name))


NAMES = Names()


class Writer(object):

    def __init__(self):
        self.out = io.StringIO()
        self.indent = 0

    def w(self, line=''):
        if line:
            self.out.write(' ' * self.indent + line + '\n')
        else:
            self.out.write('\n')

    def getvalue(self):
        return self.out.getvalue()


def eus_value(v, base):
    if base == 'bool':
        return 't' if v else 'nil'
    if base in STRING_TYPES:
        return lisp_string(v)
    if base in FLOAT_TYPES:
        return repr(float(v))
    return str(int(v))


def init_value(f, names):
    """Default value of a slot."""
    if f.is_primitive:
        base = f.base
        if f.default is not None:
            if f.is_array:
                vals = f.default
                if base in BYTE_TYPES:
                    return '(coerce (list %s) string)' % ' '.join(
                        str(int(v)) for v in vals)
                if base in INTEGER_TYPES:
                    return '(integer-vector %s)' % ' '.join(
                        str(int(v)) for v in vals)
                if base in FLOAT_TYPES:
                    return '(float-vector %s)' % ' '.join(
                        repr(float(v)) for v in vals)
                return '(list %s)' % ' '.join(eus_value(v, base) for v in vals)
            return eus_value(f.default, base)
        if f.is_array:
            n = f.array_len or 0
            if base in BYTE_TYPES:
                return '(make-string %d)' % n
            if base in INTEGER_TYPES:
                return '(instantiate integer-vector %d)' % n
            if base in FLOAT_TYPES:
                return '(instantiate float-vector %d)' % n
            if base == 'bool':
                return '(make-list %d)' % n
            return '(make-list %d :initial-element "")' % n
        if base == 'bool':
            return 'nil'
        if base in FLOAT_TYPES:
            return '0.0'
        if base in STRING_TYPES:
            return '""'
        return '0'
    cls = f.eus_class(names)
    if f.is_array:
        if f.array_len:
            return '(let (r) (dotimes (i %d) (push (instance %s :init) r)) r)' % (
                f.array_len, cls)
        return 'nil'
    return '(instance %s :init)' % cls


def init_form(f):
    var = '__%s' % f.name
    if f.is_primitive and not f.is_array:
        if f.base in INTEGER_TYPES:
            return '(round %s)' % var
        if f.base in FLOAT_TYPES:
            return '(float %s)' % var
        if f.base in STRING_TYPES:
            return '(string %s)' % var
    return var


# serialization helpers, the slot / element value is v, p is the position

def len_primitive(base, p='p'):
    if base in STRING_TYPES:
        return None
    size = PRIMITIVES[base][0]
    if size == 1:
        return '(setq %s (+ %s 1))' % (p, p)
    return '(setq %s (+ (ros::cdr-align %s %d) %d))' % (p, p, size, size)


def len_string(base, v):
    if base == 'wstring':
        # uint32 number of characters, 4 bytes per character, no terminator
        return '(+ (ros::cdr-align p 4) 4 (* 4 (length %s)))' % v
    # uint32 length including the terminating NUL, bytes, NUL
    return '(+ (ros::cdr-align p 4) 5 (length %s))' % v


def write_serialization_length(w, fields, names):
    w.w('(:serialization-length')
    w.w(' (&optional (p 0))')
    w.indent += 1
    if not fields:
        w.w(';; structure_needs_at_least_one_member')
        w.w('(setq p (+ p 1))')
    for f in fields:
        w.w(';; %s _%s' % (f.type_str, f.name))
        slot = '_%s' % f.name
        if f.is_primitive:
            base = f.base
            size = PRIMITIVES[base][0] if base in PRIMITIVES else None
            if f.is_array:
                if not f.array_len:
                    w.w('(setq p (+ (ros::cdr-align p 4) 4))')
                if base in STRING_TYPES:
                    w.w('(dolist (e %s) (setq p %s))' % (slot, len_string(base, 'e')))
                elif size == 1:
                    w.w('(setq p (+ p (length %s)))' % slot)
                else:
                    w.w('(if (> (length %s) 0) (setq p (+ (ros::cdr-align p %d) (* %d (length %s)))))'
                        % (slot, size, size, slot))
            else:
                if base in STRING_TYPES:
                    w.w('(setq p %s)' % len_string(base, slot))
                else:
                    w.w(len_primitive(base))
        else:
            if f.is_array:
                if not f.array_len:
                    w.w('(setq p (+ (ros::cdr-align p 4) 4))')
                w.w('(dolist (e %s) (setq p (send e :serialization-length p)))' % slot)
            else:
                w.w('(setq p (send %s :serialization-length p))' % slot)
    w.w('p)')
    w.indent -= 1


def poke_primitive(base, v):
    size, ptype = PRIMITIVES[base]
    if base == 'bool':
        v = '(if %s 1 0)' % v
    elif base == 'uint64':
        v = '(ros::cdr-uint64 %s)' % v
    if size == 1:
        return '(sys::poke %s buf p :char) (incf p 1)' % v
    return ('(setq p (ros::cdr-align p %d)) (sys::poke %s buf p %s) (incf p %d)'
            % (size, v, ptype, size))


def write_serialize(w, fields, names):
    w.w('(:serialize-cdr')
    w.w(' (buf p)')
    w.indent += 1
    if not fields:
        w.w(';; structure_needs_at_least_one_member')
        w.w('(sys::poke 0 buf p :char) (incf p 1)')
    for f in fields:
        w.w(';; %s _%s' % (f.type_str, f.name))
        slot = '_%s' % f.name
        if f.is_primitive:
            base = f.base
            wide = 't' if base == 'wstring' else 'nil'
            if f.is_array:
                if not f.array_len:
                    w.w('(setq p (ros::cdr-align p 4)) (sys::poke (length %s) buf p :integer) (incf p 4)'
                        % slot)
                if base in BYTE_TYPES:
                    w.w('(replace buf %s :start1 p) (incf p (length %s))' % (slot, slot))
                elif base in STRING_TYPES:
                    w.w('(dolist (e %s) (setq p (ros::cdr-put-string buf p e %s)))' % (slot, wide))
                elif base == 'bool':
                    w.w('(dolist (e %s) (sys::poke (if e 1 0) buf p :char) (incf p 1))' % slot)
                else:
                    size, ptype = PRIMITIVES[base]
                    w.w('(when (> (length %s) 0)' % slot)
                    if size > 1:
                        w.w('  (setq p (ros::cdr-align p %d))' % size)
                    v = '(elt %s i)' % slot
                    if base == 'uint64':
                        v = '(ros::cdr-uint64 %s)' % v
                    w.w('  (dotimes (i (length %s)) (sys::poke %s buf p %s) (incf p %d)))'
                        % (slot, v, ptype, size))
            else:
                if base in STRING_TYPES:
                    w.w('(setq p (ros::cdr-put-string buf p %s %s))' % (slot, wide))
                else:
                    w.w(poke_primitive(base, slot))
        else:
            if f.is_array:
                if not f.array_len:
                    w.w('(setq p (ros::cdr-align p 4)) (sys::poke (length %s) buf p :integer) (incf p 4)'
                        % slot)
                w.w('(dolist (e %s) (setq p (send e :serialize-cdr buf p)))' % slot)
            else:
                w.w('(setq p (send %s :serialize-cdr buf p))' % slot)
    w.w('p)')
    w.indent -= 1


def peek_primitive(base):
    """Expression reading base at p (p is already aligned)."""
    size, ptype = PRIMITIVES[base]
    e = '(sys::peek buf p %s)' % ptype
    if base == 'bool':
        return '(/= 0 %s)' % e
    if base == 'int8':
        return '(ros::cdr-int8 %s)' % e
    if base == 'uint16':
        return '(logand %s #xffff)' % e
    if base == 'uint32':
        return '(ros::cdr-uint32 %s)' % e
    if base == 'uint64':
        return '(ros::cdr-to-uint64 %s)' % e
    return e


def write_deserialize(w, fields, names):
    w.w('(:deserialize-cdr')
    w.w(' (buf p)')
    w.indent += 1
    if not fields:
        w.w(';; structure_needs_at_least_one_member')
        w.w('(incf p 1)')
    for f in fields:
        w.w(';; %s _%s' % (f.type_str, f.name))
        slot = '_%s' % f.name
        if f.is_array:
            if f.array_len:
                n = str(f.array_len)
            else:
                w.w('(let (n)')
                w.indent += 1
                w.w('(setq p (ros::cdr-align p 4)) (setq n (sys::peek buf p :integer)) (incf p 4)')
                n = 'n'
            if f.is_primitive:
                base = f.base
                wide = 't' if base == 'wstring' else 'nil'
                if base in BYTE_TYPES:
                    w.w('(setq %s (subseq buf p (+ p %s))) (incf p %s)' % (slot, n, n))
                elif base in STRING_TYPES:
                    w.w('(setq %s (make-list %s))' % (slot, n))
                    w.w('(dotimes (i %s) (let ((r (ros::cdr-get-string buf p %s))) (setf (elt %s i) (car r)) (setq p (cdr r))))'
                        % (n, wide, slot))
                elif base == 'bool':
                    w.w('(setq %s (make-list %s))' % (slot, n))
                    w.w('(dotimes (i %s) (setf (elt %s i) (/= 0 (sys::peek buf p :char))) (incf p 1))'
                        % (n, slot))
                else:
                    size = PRIMITIVES[base][0]
                    vec = 'float-vector' if base in FLOAT_TYPES else 'integer-vector'
                    w.w('(setq %s (instantiate %s %s))' % (slot, vec, n))
                    w.w('(when (> %s 0)' % n)
                    if size > 1:
                        w.w('  (setq p (ros::cdr-align p %d))' % size)
                    w.w('  (dotimes (i %s) (setf (elt %s i) %s) (incf p %d)))'
                        % (n, slot, peek_primitive(base), size))
            else:
                cls = f.eus_class(names)
                w.w('(setq %s (let (r) (dotimes (i %s) (push (instance %s :init) r)) r))'
                    % (slot, n, cls))
                w.w('(dolist (e %s) (setq p (send e :deserialize-cdr buf p)))' % slot)
            if not f.array_len:
                w.w(')')
                w.indent -= 1
        else:
            if f.is_primitive:
                base = f.base
                if base in STRING_TYPES:
                    w.w('(let ((r (ros::cdr-get-string buf p %s))) (setq %s (car r)) (setq p (cdr r)))'
                        % ('t' if base == 'wstring' else 'nil', slot))
                else:
                    size = PRIMITIVES[base][0]
                    if size > 1:
                        w.w('(setq p (ros::cdr-align p %d)) (setq %s %s) (incf p %d)'
                            % (size, slot, peek_primitive(base), size))
                    else:
                        w.w('(setq %s %s) (incf p 1)' % (slot, peek_primitive(base)))
            else:
                w.w('(setq p (send %s :deserialize-cdr buf p))' % slot)
    w.w('p)')
    w.indent -= 1


def write_accessors(w, fields):
    for f in fields:
        var = '_%s' % f.name
        w.w('(:%s' % f.name)
        if f.is_primitive or f.is_array or f.is_time:
            if f.is_primitive and f.base == 'bool' and not f.is_array:
                w.w(' (&optional (__%s :null))' % f.name)
                w.w(' (if (not (eq __%s :null)) (setq %s __%s)) %s)' % (f.name, var, f.name, var))
            else:
                w.w(' (&optional __%s)' % f.name)
                w.w(' (if __%s (setq %s __%s)) %s)' % (f.name, var, f.name, var))
        else:
            w.w(' (&rest __%s)' % f.name)
            w.w(' (if (keywordp (car __%s))' % f.name)
            w.w('     (send* %s __%s)' % (var, f.name))
            w.w('   (progn')
            w.w('     (if __%s (setq %s (car __%s)))' % (f.name, var, f.name))
            w.w('     %s)))' % var)


def write_constants(w, pkg, cls, constants):
    if not constants:
        return
    pkgname = '%s::%s' % (pkg.upper(), cls.upper())
    for c in constants:
        name = c.name.upper()
        w.w('(intern "*%s*" (find-package "%s"))' % (name, pkgname))
        w.w('(shadow \'*%s* (find-package "%s"))' % (name, pkgname))
        w.w('(defconstant %s::%s::*%s* %s)' % (pkg, cls, name, eus_value(c.value, c.type)))
    w.w('')
    w.w('(defun %s::%s-to-symbol (const)' % (pkg, cls))
    w.w('  (cond')
    for c in constants:
        if c.type in INTEGER_TYPES:
            w.w("        ((= const %s) '%s::%s::*%s*)" % (int(c.value), pkg, cls, c.name.upper()))
    w.w('        (t nil)))')
    w.w('')


def write_package_begin(w, pkg, cls):
    w.w('(when (boundp \'%s::%s)' % (pkg, cls))
    w.w('  (if (not (find-package "%s"))' % pkg.upper())
    w.w('    (make-package "%s"))' % pkg.upper())
    w.w('  (shadow \'%s (find-package "%s")))' % (cls, pkg.upper()))
    w.w('(unless (find-package "%s::%s")' % (pkg.upper(), cls.upper()))
    w.w('  (make-package "%s::%s"))' % (pkg.upper(), cls.upper()))


def write_requires(w, pkg, specs):
    deps = set()
    for spec_fields in specs:
        for f in spec_fields:
            if f.msg and not f.is_time and f.msg[0] != pkg:
                deps.add(f.msg[0])
    for d in sorted(deps):
        w.w('(if (not (find-package "%s"))' % d.upper())
        w.w('  (ros::roseus-add-msgs "%s"))' % d)


def write_class(w, pkg, cls, fields, constants, datatype, definition, type_hash,
                names):
    """Write defclass, defmethod and the properties of one message class."""
    write_constants(w, pkg, cls, constants)
    w.w('(defclass %s::%s' % (pkg, cls))
    w.w('  :super ros::object')
    w.w('  :slots (%s))' % ''.join('_%s ' % f.name for f in fields))
    w.w('')
    w.w('(defmethod %s::%s' % (pkg, cls))
    w.indent += 2
    w.w('(:init')
    w.w(' (&key')
    for f in fields:
        w.w('  ((:%s __%s) %s)' % (f.name, f.name, init_value(f, names)))
    w.w('  )')
    w.w(' (send-super :init)')
    for f in fields:
        w.w(' (setq _%s %s)' % (f.name, init_form(f)))
    w.w(' self)')
    write_accessors(w, fields)
    write_serialization_length(w, fields, names)
    write_serialize(w, fields, names)
    write_deserialize(w, fields, names)
    w.indent -= 2
    w.w('  )')
    w.w('')
    w.w('(setf (get %s::%s :md5sum-) %s)' % (pkg, cls, lisp_string(type_hash)))
    w.w('(setf (get %s::%s :datatype-) %s)' % (pkg, cls, lisp_string(datatype)))
    w.w('(setf (get %s::%s :definition-)' % (pkg, cls))
    w.w('      %s)' % lisp_string(definition))
    w.w('')


def type_hash(share_dir, sub, name):
    """RIHS type hash from the type description json, if any."""
    if not share_dir:
        return ''
    path = os.path.join(share_dir, sub, name + '.json')
    try:
        with open(path) as f:
            desc = json.load(f)
        for h in desc.get('type_hashes', []):
            if h.get('type_name') == desc['type_description_msg']['type_description']['type_name']:
                return h.get('hash_string', '')
    except Exception:
        pass
    return ''


def header(w):
    w.w(';; Auto-generated by rosidl_generator_eus. Do not edit!')
    w.w('')


def generate_msg(pkg, name, text, share_dir):
    spec = parse_message_string(pkg, name, text)
    fields = [Field(f, pkg) for f in spec.fields]
    w = Writer()
    header(w)
    if (pkg, name) in TIME_TYPES:
        # builtin_interfaces/Time and Duration are ros::time / ros::duration
        cls = name
        base = TIME_TYPES[(pkg, name)]
        write_package_begin(w, pkg, cls)
        w.w('')
        w.w('(in-package "ROS")')
        w.w('')
        w.w('(defclass %s::%s' % (pkg, cls))
        w.w('  :super %s' % base)
        w.w('  :slots ())')
        w.w('')
        w.w('(setf (get %s::%s :md5sum-) %s)' % (pkg, cls,
                                                 lisp_string(type_hash(share_dir, 'msg', name))))
        w.w('(setf (get %s::%s :datatype-) "%s/msg/%s")' % (pkg, cls, pkg, name))
        w.w('(setf (get %s::%s :definition-)' % (pkg, cls))
        w.w('      %s)' % lisp_string(text))
        w.w('')
    else:
        cls = name
        write_package_begin(w, pkg, cls)
        w.w('')
        w.w('(in-package "ROS")')
        write_requires(w, pkg, [fields])
        w.w('')
        write_class(w, pkg, cls, fields, spec.constants, '%s/msg/%s' % (pkg, name),
                    text, type_hash(share_dir, 'msg', name), NAMES)
    w.w('(provide :%s/%s "%s")' % (pkg, name, type_hash(share_dir, 'msg', name)))
    w.w('')
    return w.getvalue()


def generate_srv(pkg, name, text, share_dir):
    spec = parse_service_string(pkg, name, text)
    parts = text.split('---')
    w = Writer()
    header(w)
    write_srv(w, pkg, name, 'srv',
              [Field(f, pkg) for f in spec.request.fields], spec.request.constants,
              parts[0],
              [Field(f, pkg) for f in spec.response.fields], spec.response.constants,
              parts[1] if len(parts) > 1 else '',
              text, type_hash(share_dir, 'srv', name))
    w.w('(provide :%s/%s "%s")' % (pkg, name, type_hash(share_dir, 'srv', name)))
    w.w('')
    return w.getvalue()


def write_srv(w, pkg, name, sub, req_fields, req_constants, req_def,
              res_fields, res_constants, res_def, text, srv_hash):
    req_cls = name + 'Request'
    res_cls = name + 'Response'
    for c in (name, req_cls, res_cls):
        write_package_begin(w, pkg, c)
    w.w('')
    w.w('(in-package "ROS")')
    write_requires(w, pkg, [req_fields, res_fields])
    w.w('')
    write_class(w, pkg, req_cls, req_fields, req_constants,
                '%s/%s/%s_Request' % (pkg, sub, name), req_def, srv_hash, NAMES)
    write_class(w, pkg, res_cls, res_fields, res_constants,
                '%s/%s/%s_Response' % (pkg, sub, name), res_def, srv_hash, NAMES)
    w.w('(defclass %s::%s' % (pkg, name))
    w.w('  :super ros::object')
    w.w('  :slots ())')
    w.w('')
    w.w('(setf (get %s::%s :md5sum-) %s)' % (pkg, name, lisp_string(srv_hash)))
    w.w('(setf (get %s::%s :datatype-) "%s/%s/%s")' % (pkg, name, pkg, sub, name))
    w.w('(setf (get %s::%s :request) %s::%s)' % (pkg, name, pkg, req_cls))
    w.w('(setf (get %s::%s :response) %s::%s)' % (pkg, name, pkg, res_cls))
    w.w('(setf (get %s::%s :definition-)' % (pkg, name))
    w.w('      %s)' % lisp_string(text))
    w.w('')
    w.w('(defmethod %s::%s' % (pkg, req_cls))
    w.w('  (:response () (instance %s::%s :init)))' % (pkg, res_cls))
    w.w('')


def generate_action(pkg, name, text, share_dir):
    """Generate the classes of an action.

    The goal / result / feedback are pkg::NameGoal, pkg::NameResult and
    pkg::NameFeedback (the names of the ROS 1 roseus), the types of the
    ROS 2 action protocol are pkg::Name_SendGoal{,Request,Response},
    pkg::Name_GetResult{,Request,Response} and pkg::Name_FeedbackMessage.
    """
    spec = parse_action_string(pkg, name, text)
    for suffix in ('Goal', 'Result', 'Feedback'):
        NAMES.alias[(pkg, name + '_' + suffix)] = name + suffix
    parts = text.split('---')
    while len(parts) < 3:
        parts.append('')
    h = type_hash(share_dir, 'action', name)
    w = Writer()
    header(w)
    for suffix, sub_spec, definition in (('Goal', spec.goal, parts[0]),
                                         ('Result', spec.result, parts[1]),
                                         ('Feedback', spec.feedback, parts[2])):
        fields = [Field(f, pkg) for f in sub_spec.fields]
        cls = name + suffix
        write_package_begin(w, pkg, cls)
        w.w('')
        w.w('(in-package "ROS")')
        write_requires(w, pkg, [fields])
        w.w('')
        write_class(w, pkg, cls, fields, sub_spec.constants,
                    '%s/action/%s_%s' % (pkg, name, suffix), definition, h, NAMES)
    # the services and the feedback message of the action protocol
    uuid = Field.make('goal_id', msg=('unique_identifier_msgs', 'UUID'))
    goal_def = 'unique_identifier_msgs/UUID goal_id\n%s/%s_Goal goal\n' % (pkg, name)
    send_goal_res_def = 'bool accepted\nbuiltin_interfaces/Time stamp\n'
    write_srv(w, pkg, name + '_SendGoal', 'action',
              [uuid, Field.make('goal', msg=(pkg, name + '_Goal'))], [], goal_def,
              [Field.make('accepted', base='bool'),
               Field.make('stamp', msg=('builtin_interfaces', 'Time'))], [],
              send_goal_res_def,
              goal_def + '---\n' + send_goal_res_def, h)
    get_result_req_def = 'unique_identifier_msgs/UUID goal_id\n'
    get_result_res_def = 'int8 status\n%s/%s_Result result\n' % (pkg, name)
    write_srv(w, pkg, name + '_GetResult', 'action',
              [uuid], [], get_result_req_def,
              [Field.make('status', base='int8'),
               Field.make('result', msg=(pkg, name + '_Result'))], [],
              get_result_res_def,
              get_result_req_def + '---\n' + get_result_res_def, h)
    fb_def = 'unique_identifier_msgs/UUID goal_id\n%s/%s_Feedback feedback\n' % (pkg, name)
    fb_fields = [uuid, Field.make('feedback', msg=(pkg, name + '_Feedback'))]
    write_package_begin(w, pkg, name + '_FeedbackMessage')
    w.w('')
    w.w('(in-package "ROS")')
    write_requires(w, pkg, [fb_fields])
    w.w('')
    write_class(w, pkg, name + '_FeedbackMessage', fb_fields, [],
                '%s/action/%s_FeedbackMessage' % (pkg, name), fb_def, h, NAMES)
    # the action class itself
    write_package_begin(w, pkg, name)
    w.w('(defclass %s::%s' % (pkg, name))
    w.w('  :super ros::object')
    w.w('  :slots ())')
    w.w('(setf (get %s::%s :datatype-) "%s/action/%s")' % (pkg, name, pkg, name))
    w.w('(setf (get %s::%s :md5sum-) %s)' % (pkg, name, lisp_string(h)))
    for key, cls in (('goal', 'Goal'), ('result', 'Result'), ('feedback', 'Feedback'),
                     ('send-goal', '_SendGoal'), ('get-result', '_GetResult'),
                     ('feedback-message', '_FeedbackMessage')):
        w.w('(setf (get %s::%s :%s) %s::%s%s)' % (pkg, name, key, pkg, name, cls))
    w.w('(setf (get %s::%s :definition-)' % (pkg, name))
    w.w('      %s)' % lisp_string(text))
    w.w('')
    # the names of the ROS 1 roseus: pkg::FooAction, FooActionGoal, ...
    for alias, cls in (('Action', ''), ('ActionGoal', '_SendGoalRequest'),
                       ('ActionResult', '_GetResultResponse'),
                       ('ActionFeedback', '_FeedbackMessage')):
        w.w('(setq %s::%s%s %s::%s%s)' % (pkg, name, alias, pkg, name, cls))
    w.w('')
    w.w('(provide :%s/%s "%s")' % (pkg, name, h))
    w.w('')
    return w.getvalue()


def find_share_dir(pkg):
    try:
        from ament_index_python.packages import get_package_share_directory
        return get_package_share_directory(pkg)
    except Exception:
        for prefix in os.environ.get('AMENT_PREFIX_PATH', '').split(os.pathsep):
            d = os.path.join(prefix, 'share', pkg)
            if prefix and os.path.isdir(d):
                return d
    return None


def interface_files(src_dir):
    ret = []
    for sub, ext in (('msg', '.msg'), ('srv', '.srv'), ('action', '.action')):
        for path in sorted(glob.glob(os.path.join(src_dir, sub, '*' + ext))):
            ret.append((sub, os.path.splitext(os.path.basename(path))[0], path))
    return ret


def generate_package(pkg, out_dir, src_dir=None, share_dir=None, files=None):
    """Generate the files of pkg into out_dir/pkg. Returns number of files.

    files is a list of the interface files (.../msg/Foo.msg etc.), when it
    is None, the files in src_dir (or the share directory of pkg) are used.
    """
    if share_dir is None:
        share_dir = find_share_dir(pkg)
    if files is not None:
        files = [(os.path.basename(os.path.dirname(f)),
                  os.path.splitext(os.path.basename(f))[0], f)
                 for f in files if os.path.splitext(f)[1] in ('.msg', '.srv', '.action')]
    else:
        if src_dir is None:
            src_dir = share_dir
        if src_dir is None:
            raise RuntimeError('could not find package %s' % pkg)
        files = interface_files(src_dir)
    # register action aliases first, they may be used by other files
    for sub, name, _ in files:
        if sub == 'action':
            for suffix in ('Goal', 'Result', 'Feedback'):
                NAMES.alias[(pkg, name + '_' + suffix)] = name + suffix
    pkg_out = os.path.join(out_dir, pkg)
    parent = os.path.dirname(os.path.abspath(pkg_out))
    if not os.path.isdir(parent):
        os.makedirs(parent)
    # write into a temporary directory, and rename it, so that roseus
    # processes generating the same package at the same time never see
    # a half generated package.
    tmp = tempfile.mkdtemp(prefix='.%s-' % pkg, dir=parent)
    try:
        count = 0
        for sub, name, path in files:
            with io.open(path, encoding='utf-8') as f:
                text = f.read()
            try:
                if sub == 'msg':
                    out = generate_msg(pkg, name, text, share_dir)
                elif sub == 'srv':
                    out = generate_srv(pkg, name, text, share_dir)
                else:
                    out = generate_action(pkg, name, text, share_dir)
            except Exception as e:
                sys.stderr.write('[gen_eus_ros2] failed to generate %s/%s/%s: %s\n'
                                 % (pkg, sub, name, e))
                continue
            d = os.path.join(tmp, sub)
            if not os.path.isdir(d):
                os.makedirs(d)
            with io.open(os.path.join(d, name + '.l'), 'w', encoding='utf-8') as f:
                f.write(out)
            count += 1
        os.chmod(tmp, 0o755)
        if os.path.isdir(pkg_out):
            old = tempfile.mkdtemp(prefix='.%s-old-' % pkg, dir=parent)
            os.rename(pkg_out, os.path.join(old, pkg))
            os.rename(tmp, pkg_out)
            shutil.rmtree(old, ignore_errors=True)
        else:
            try:
                os.rename(tmp, pkg_out)
            except OSError:
                # another process generated it in the meantime
                shutil.rmtree(tmp, ignore_errors=True)
    except Exception:
        shutil.rmtree(tmp, ignore_errors=True)
        raise
    return count


def main(argv=None):
    parser = argparse.ArgumentParser(
        description='generate roseus message classes for ROS 2 interfaces')
    parser.add_argument('packages', nargs='+', metavar='PKG')
    parser.add_argument('-o', '--output-dir', default=None,
                        help='output directory (default: $ROS_HOME/roseus/$ROS_DISTRO)')
    parser.add_argument('--source', action='append', default=[], metavar='PKG:DIR',
                        help='read the interface files of PKG from DIR')
    parser.add_argument('--share', action='append', default=[], metavar='PKG:DIR',
                        help='share directory of PKG (for the type hashes)')
    parser.add_argument('--files', nargs='+', default=None, metavar='FILE',
                        help='interface files (.../msg/Foo.msg) of the (only one) PKG')
    args = parser.parse_args(argv)
    if args.files is not None and len(args.packages) != 1:
        parser.error('--files requires only one PKG')
    out_dir = args.output_dir
    if out_dir is None:
        ros_home = os.environ.get('ROS_HOME', os.path.join(os.path.expanduser('~'), '.ros'))
        out_dir = os.path.join(ros_home, 'roseus', os.environ.get('ROS_DISTRO', 'unknown'))
    sources = dict(s.split(':', 1) for s in args.source)
    shares = dict(s.split(':', 1) for s in args.share)
    ret = 0
    for pkg in args.packages:
        try:
            n = generate_package(pkg, out_dir, sources.get(pkg), shares.get(pkg), args.files)
            print('[gen_eus_ros2] %s: %d files -> %s' % (pkg, n, os.path.join(out_dir, pkg)))
        except Exception as e:
            sys.stderr.write('[gen_eus_ros2] %s: %s\n' % (pkg, e))
            ret = 1
    return ret


if __name__ == '__main__':
    sys.exit(main())
