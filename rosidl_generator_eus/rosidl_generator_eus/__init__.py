# Software License Agreement (BSD License)
#
# Copyright (c) 2026, JSK Robotics Laboratory.
# Copyright (c) 2026, Yoshiki Obinata.
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
#  * Neither the name of JSK Robotics Laboratory. nor the names of its
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

"""ROS 2 EusLisp code generator using rosidl pipeline."""

import os
import pathlib
from io import StringIO

from rosidl_pycommon import read_generator_arguments

from rosidl_parser.definition import (
    AbstractGenericString,
    AbstractNestedType,
    AbstractSequence,
    Action,
    Array,
    BasicType,
    BoundedSequence,
    Message,
    NamespacedType,
    Service,
    UnboundedSequence,
)
from rosidl_parser.parser import parse_idl_file
from rosidl_parser.definition import IdlLocator


# Copied from geneus.generate:
# https://github.com/jsk-ros-pkg/geneus/blob/master/src/geneus/generate.py
class IndentedWriter:
    """Write indented Lisp forms.

    This used to be imported from the ROS 1-only ``geneus`` package.  Keeping
    the tiny formatting helper here makes the ROS 2 generator self-contained.
    """

    def __init__(self, stream):
        self.str = stream
        self.indentation = 0
        self.block_indent = False

    def write(self, value, indent=True, newline=True):
        if not indent:
            newline = False
        if self.block_indent:
            self.block_indent = False
        else:
            if newline:
                self.str.write('\n')
            if indent:
                self.str.write(' ' * self.indentation)
        self.str.write(value)

    def newline(self):
        self.str.write('\n')

    def inc_indent(self, amount=2):
        self.indentation += amount

    def dec_indent(self, amount=2):
        self.indentation -= amount

    def block_next_indent(self):
        self.block_indent = True


class Indent:
    """Context manager for :class:`IndentedWriter` indentation."""

    def __init__(self, writer, inc=2, indent_first=True):
        self.writer = writer
        self.inc = inc
        self.indent_first = indent_first

    def __enter__(self):
        self.writer.inc_indent(self.inc)
        if not self.indent_first:
            self.writer.block_next_indent()

    def __exit__(self, _type, _value, _traceback):
        self.writer.dec_indent(self.inc)


# ============================================================
# Type helpers for ROS 2 IDL basic types
# ============================================================

# Byte sizes for serialization
NUM_BYTES = {
    'int8': 1, 'uint8': 1, 'octet': 1, 'char': 1, 'boolean': 1,
    'int16': 2, 'uint16': 2, 'wchar': 2,
    'short': 2, 'unsigned short': 2,
    'int32': 4, 'uint32': 4,
    'long': 4, 'unsigned long': 4,
    'float': 4,
    'int64': 8, 'uint64': 8,
    'long long': 8, 'unsigned long long': 8,
    'double': 8, 'long double': 8,
}

_SIGNED_INT_TYPES = frozenset((
    'int8', 'int16', 'int32', 'int64',
    'short', 'long', 'long long',
))

_UNSIGNED_INT_TYPES = frozenset((
    'uint8', 'uint16', 'uint32', 'uint64',
    'octet', 'char', 'wchar',
    'unsigned short', 'unsigned long', 'unsigned long long',
))

_INTEGER_TYPES = _SIGNED_INT_TYPES | _UNSIGNED_INT_TYPES

_FLOAT_TYPES = frozenset(('float', 'double', 'long double'))


def _is_integer(t):
    return t in _INTEGER_TYPES


def _is_signed_int(t):
    return t in _SIGNED_INT_TYPES


def _is_unsigned_int(t):
    return t in _UNSIGNED_INT_TYPES


def _is_bool(t):
    return t == 'boolean'


def _is_float(t):
    return t in _FLOAT_TYPES


def _is_string(t):
    return t == 'string'


# ============================================================
# Field adapter: bridges rosidl_parser Member → geneus-like field
# ============================================================

class FieldInfo:
    """Adapter wrapping a rosidl_parser Member into a geneus-compatible field."""

    def __init__(self, member):
        self.name = member.name
        type_ = member.type

        # Unwrap nested types (Array / Sequence)
        if isinstance(type_, Array):
            self.is_array = True
            self.array_len = type_.size
            inner = type_.value_type
        elif isinstance(type_, (BoundedSequence, UnboundedSequence)):
            self.is_array = True
            self.array_len = None  # dynamic length
            inner = type_.value_type
        else:
            self.is_array = False
            self.array_len = None
            inner = type_

        # Classify the inner (element) type
        if isinstance(inner, BasicType):
            self.is_builtin = True
            self.base_type = inner.typename
        elif isinstance(inner, AbstractGenericString):
            self.is_builtin = True
            self.base_type = 'string'
        elif isinstance(inner, NamespacedType):
            self.is_builtin = False
            self.base_type = None
            self.pkg = inner.namespaces[0]
            self.msg_name = inner.name
        else:
            raise ValueError(f'Unknown inner type: {inner}')

        # Human-readable type string (for comments)
        if self.is_builtin:
            base = self.base_type
        else:
            base = f'{self.pkg}/{self.msg_name}'
        if self.is_array:
            if self.array_len is not None:
                self.type_str = f'{base}[{self.array_len}]'
            else:
                self.type_str = f'{base}[]'
        else:
            self.type_str = base


# ============================================================
# EusLisp type mapping helpers
# ============================================================

def _lisp_type(t, is_array):
    """Map IDL basic type to EusLisp type name."""
    if t in ('uint8', 'octet', 'char') and is_array:
        return 'char'
    if _is_integer(t):
        return 'integer'
    if _is_bool(t):
        return 'object'
    if _is_float(t):
        return 'float'
    if _is_string(t):
        return 'string'
    raise ValueError(f'{t} is not a recognized primitive type')


def _lisp_initvalue(t):
    """Default EusLisp value for an IDL basic type."""
    if _is_integer(t):
        return '0'
    if _is_bool(t):
        return 'nil'
    if _is_float(t):
        return '0.0'
    if _is_string(t):
        return '""'
    raise ValueError(f'{t} is not a recognized primitive type')


def _field_type(f):
    """EusLisp type expression for a field."""
    if f.is_builtin:
        return _lisp_type(f.base_type, f.is_array)
    return f'{f.pkg}::{f.msg_name}'


def _field_initvalue(f):
    """EusLisp initialization expression for a field."""
    if not f.is_builtin:
        ft = _field_type(f)
        if f.is_array:
            if f.array_len:
                return (
                    f'(let (r) (dotimes (i {f.array_len})'
                    f' (push (instance {ft} :init) r)) r)')
            return '()'
        return f'(instance {ft} :init)'

    initvalue = _lisp_initvalue(f.base_type)
    elt_type = _lisp_type(f.base_type, f.is_array)
    if f.is_array:
        length = f.array_len or 0
        if not _is_string(f.base_type) and not _is_bool(f.base_type):
            return (
                f'(make-array {length} :initial-element {initvalue}'
                f' :element-type :{elt_type})')
        return f'(let (r) (dotimes (i {length}) (push {initvalue} r)) r)'
    return initvalue


def _field_initform(f):
    """EusLisp setq form for :init method."""
    var = f'__{f.name}'
    if f.is_builtin and not f.is_array:
        if _is_integer(f.base_type):
            return f'(round {var})'
        if _is_float(f.base_type):
            return f'(float {var})'
        if _is_string(f.base_type):
            return f'(string {var})'
    return var


# ============================================================
# Code generation: write_* functions
# ============================================================

def _write_begin(s, pkg, name, is_service=False):
    """Write file header and package declarations."""
    s.write(';; Auto-generated. Do not edit!\n\n', newline=False)
    s.write(f'(when (boundp \'{pkg}::{name})')
    s.write(f'  (if (not (find-package "{pkg.upper()}"))')
    s.write(f'    (make-package "{pkg.upper()}"))')
    s.write(f'  (shadow \'{name} (find-package "{pkg.upper()}")))')
    s.write(f'(unless (find-package "{pkg.upper()}::{name.upper()}")')
    s.write(f'  (make-package "{pkg.upper()}::{name.upper()}"))')
    if is_service:
        s.write(
            f'(unless (find-package'
            f' "{pkg.upper()}::{name.upper()}REQUEST")')
        s.write(
            f'  (make-package'
            f' "{pkg.upper()}::{name.upper()}REQUEST"))')
        s.write(
            f'(unless (find-package'
            f' "{pkg.upper()}::{name.upper()}RESPONSE")')
        s.write(
            f'  (make-package'
            f' "{pkg.upper()}::{name.upper()}RESPONSE"))')
    s.write('')
    s.write('(in-package "ROS")')
    s.newline()


def _write_include(s, pkg, fields):
    """Write dependency loading for cross-package types."""
    dep_pkgs = sorted(set(
        f.pkg for f in fields
        if not f.is_builtin and f.pkg != pkg))
    for dep_pkg in dep_pkgs:
        s.write(f'(if (not (find-package "{dep_pkg.upper()}"))')
        s.write(f'  (ros::roseus-add-msgs "{dep_pkg}"))')
    s.newline()
    s.newline()


def _write_constants(s, pkg, name, constants):
    """Write constant definitions."""
    if not constants:
        return
    for c in constants:
        c_name = c.name.upper()
        s.write(
            f'(intern "*{c_name}*"'
            f' (find-package "{pkg.upper()}::{name.upper()}"))')
        s.write(
            f'(shadow \'*{c_name}*'
            f' (find-package "{pkg.upper()}::{name.upper()}"))')
        if isinstance(c.type, AbstractGenericString):
            escaped = str(c.value).replace('\\', '\\\\').replace('"', '\\"')
            s.write(
                f'(defconstant {pkg}::{name}::*{c_name}* "{escaped}")')
        elif isinstance(c.type, BasicType) and _is_bool(c.type.typename):
            val = 't' if c.value else 'nil'
            s.write(f'(defconstant {pkg}::{name}::*{c_name}* {val})')
        else:
            s.write(
                f'(defconstant {pkg}::{name}::*{c_name}* {c.value})')
    s.write('')
    s.write(f'(defun {pkg}::{name}-to-symbol (const)')
    s.write('  (cond')
    for c in constants:
        if isinstance(c.type, BasicType) and _is_integer(c.type.typename):
            s.write(
                f"        ((= const {c.value})"
                f" '{pkg}::{name}::*{c.name.upper()}*)")
    s.write('        (t nil)))')
    s.write('')


def _write_defclass(s, pkg, name, fields):
    """Write defclass form."""
    s.write(f'(defclass {pkg}::{name}')
    with Indent(s):
        s.write(':super ros::object')
        s.write(':slots (')
        with Indent(s, inc=1, indent_first=False):
            for f in fields:
                s.write(f'_{f.name} ', indent=False, newline=False)
        s.write('))', indent=False)
    s.newline()


def _write_defmethod(s, pkg, name, fields):
    """Write :init defmethod."""
    s.write(f'(defmethod {pkg}::{name}')
    with Indent(s):
        s.write('(:init')
        with Indent(s, inc=1):
            s.write('(&key')
            with Indent(s, inc=1):
                for f in fields:
                    s.write(
                        f'((:{f.name} __{f.name})'
                        f' {_field_initvalue(f)})')
                s.write(')')
            s.write('(send-super :init)')
            for f in fields:
                s.write(f'(setq _{f.name} {_field_initform(f)})')
            s.write('self)')


def _write_accessors(s, fields):
    """Write accessor methods."""
    with Indent(s):
        for f in fields:
            s.write(f'(:{f.name}')
            var = f'_{f.name}'
            with Indent(s, inc=1):
                if f.is_builtin:
                    if f.base_type == 'boolean':
                        s.write(f'(&optional (_{var} :null))')
                        s.write(
                            f'(if (not (eq _{var} :null))'
                            f' (setq {var} _{var})) {var})')
                    else:
                        s.write(f'(&optional _{var})')
                        s.write(
                            f'(if _{var} (setq {var} _{var})) {var})')
                else:
                    s.write(f'(&rest _{var})')
                    s.write(f'(if (keywordp (car _{var}))')
                    s.write(f'    (send* {var} _{var})')
                    with Indent(s, inc=2):
                        s.write('(progn')
                        s.write(
                            f'  (if _{var}'
                            f' (setq {var} (car _{var})))')
                        s.write(f'  {var})))')


# -- Serialization --

def _write_serialize_length(s, v, is_array=False):
    if is_array:
        s.write(f'(write-long (length {v}) s)')
    else:
        s.write(f'(write-long (length {v}) s) (princ {v} s)')


def _write_serialize_bits(s, v, num_bytes):
    if num_bytes == 1:
        s.write(f'(write-byte {v} s)')
    elif num_bytes == 2:
        s.write(f'(write-word {v} s)')
    elif num_bytes == 4:
        s.write(f'(write-long {v} s)')
    else:
        s.write('\n', indent=False)
        s.write('#+(or :alpha :irix6 :x86_64)', indent=False, newline=False)
        s.write(
            f'(progn (sys::poke {v} (send s :buffer)'
            f' (send s :count) :long)'
            f' (incf (stream-count s) 8))')
        s.write('\n', indent=False)
        s.write('#-(or :alpha :irix6 :x86_64)', indent=False)
        s.write(
            f'(cond ((and (class {v})'
            f' (= (length ({v} . bv)) 2)) ;; bignum')
        s.write(
            f'       (write-long (ash (elt ({v} . bv) 0) 0) s)')
        s.write(
            f'       (write-long (ash (elt ({v} . bv) 1) -1) s))')
        s.write(
            f'      ((and (class {v})'
            f' (= (length ({v} . bv)) 1)) ;; big1')
        s.write(f'       (write-long (elt ({v} . bv) 0) s)')
        s.write(
            f'       (write-long'
            f' (if (>= {v} 0) 0 #xffffffff) s))')
        s.write(
            '      (t                                         ;; integer')
        s.write(
            f'       (write-long {v} s)'
            f'(write-long (if (>= {v} 0) 0 #xffffffff) s)))')


def _write_serialize_builtin(s, f, v):
    if _is_string(f.base_type):
        _write_serialize_length(s, v)
    elif f.base_type == 'float':
        s.write(
            f'(sys::poke {v} (send s :buffer)'
            f' (send s :count) :float)'
            f' (incf (stream-count s) 4)')
    elif f.base_type == 'double' or f.base_type == 'long double':
        s.write(
            f'(sys::poke {v} (send s :buffer)'
            f' (send s :count) :double)'
            f' (incf (stream-count s) 8)')
    elif _is_bool(f.base_type):
        s.write(f'(if {v} (write-byte -1 s) (write-byte 0 s))')
    elif f.base_type in ('octet', 'char'):
        s.write(f'(write-byte {v} s)')
    elif _is_signed_int(f.base_type):
        _write_serialize_bits(s, v, NUM_BYTES[f.base_type])
    elif _is_unsigned_int(f.base_type):
        _write_serialize_bits(s, v, NUM_BYTES[f.base_type])
    else:
        raise ValueError(f'Unknown type: {f.base_type}')


def _write_serialize_field(s, f):
    s.write(f';; {f.type_str} _{f.name}')
    slot = f'_{f.name}'
    var = slot
    if f.is_array and f.base_type in ('uint8', 'octet', 'char'):
        if not f.array_len:
            s.write(f'(write-long (length {slot}) s)')
        s.write(f'(princ {slot} s)')
    elif f.is_array and _is_string(f.base_type):
        s.write(f'(write-long (length {slot}) s)')
        s.write(f'(dolist (elem {slot})')
        var = 'elem'
    elif f.is_array:
        if not f.array_len:
            _write_serialize_length(s, slot, True)
        if f.is_builtin and f.array_len:
            s.write(f'(dotimes (i {f.array_len})')
        elif f.is_builtin and not f.array_len:
            s.write(f'(dotimes (i (length {var}))')
        else:
            s.write(f'(dolist (elem {slot})')
        slot = 'elem'
        var = f'(elt {var} i)' if f.is_builtin else slot
        s.block_next_indent()
        s.write('')

    if f.is_array and f.base_type in ('uint8', 'octet', 'char'):
        pass
    elif f.is_builtin:
        with Indent(s):
            _write_serialize_builtin(s, f, var)
    else:
        with Indent(s):
            s.write(f'(send {slot} :serialize s)')

    if f.is_array and f.base_type not in ('uint8', 'octet', 'char'):
        s.write('  )')


def _write_serialize(s, fields):
    """Write :serialize method."""
    with Indent(s):
        s.write('(:serialize')
        with Indent(s, inc=1):
            s.write('(&optional strm)')
            s.write('(let ((s (if strm strm')
            s.write(
                '           (make-string-output-stream'
                ' (send self :serialization-length)))))')
            with Indent(s):
                for f in fields:
                    _write_serialize_field(s, f)
                s.write(';;')
                s.write(
                    '(if (null strm)'
                    ' (get-output-stream-string s))))')


# -- Deserialization --

def _write_deserialize_length(s, f, v, is_array=False):
    if is_array:
        ft = _field_type(f)
        s.write('(let (n)')
        with Indent(s):
            s.write(
                '(setq n (sys::peek buf ptr- :integer))'
                ' (incf ptr- 4)')
            s.write(
                f'(setq {v} (let (r) (dotimes (i n)'
                f' (push (instance {ft} :init) r)) r))')
    else:
        setter = 'setf' if v[0] == '(' else 'setq'
        s.write(
            f'(let (n) (setq n (sys::peek buf ptr- :integer))'
            f' (incf ptr- 4)'
            f' ({setter} {v} (subseq buf ptr- (+ ptr- n)))'
            f' (incf ptr- n))')


def _write_deserialize_bits(s, v, num_bytes):
    if num_bytes == 8:
        s.write('')
        return _write_deserialize_bits_signed(s, v, num_bytes)

    type_map = {1: ':char', 2: ':short', 4: ':integer'}
    peek_type = type_map.get(num_bytes)
    if peek_type is None:
        raise ValueError(f'Unknown size: {num_bytes}')

    setter = 'setf' if v[0] == '(' else 'setq'
    s.write(
        f'({setter} {v} (sys::peek buf ptr- {peek_type}))'
        f' (incf ptr- {num_bytes})')


def _write_deserialize_bits_signed(s, v, num_bytes):
    if num_bytes in (1, 2, 4):
        _write_deserialize_bits(s, v, num_bytes)
    else:
        s.write('\n', indent=False)
        s.write('#+(or :alpha :irix6 :x86_64)', indent=False)
        s.write(
            f' (setf {v} (prog1 (sys::peek buf ptr- :long)'
            f' (incf ptr- 8)))\n')
        s.write('#-(or :alpha :irix6 :x86_64)', indent=False)
        s.write(
            f' (setf {v}'
            f' (let ((b0 (prog1 (sys::peek buf ptr- :integer)'
            f' (incf ptr- 4)))')
        s.write(
            '             (b1 (prog1'
            ' (sys::peek buf ptr- :integer) (incf ptr- 4))))')
        s.write(f'         (cond ((= b1 -1) b0)')
        s.write(
            '                ((and (= b1  0)')
        s.write(
            '                      (<= lisp::most-negative-fixnum'
            ' b0 lisp::most-positive-fixnum))')
        s.write('                 b0)')
        s.write(
            '               ((= b1  0)'
            ' (make-instance bignum :size 1'
            ' :bv (integer-vector b0)))')
        s.write(
            '               (t (make-instance bignum :size 2'
            ' :bv (integer-vector b0 (ash b1 1)))))))')


def _write_deserialize_builtin(s, f, v):
    setter = 'setf' if v[0] == '(' else 'setq'
    if _is_string(f.base_type):
        _write_deserialize_length(s, f, v)
    elif f.base_type == 'float':
        s.write(
            f'({setter} {v} (sys::peek buf ptr- :float))'
            f' (incf ptr- 4)')
    elif f.base_type in ('double', 'long double'):
        s.write(
            f'({setter} {v} (sys::peek buf ptr- :double))'
            f' (incf ptr- 8)')
    elif _is_bool(f.base_type):
        s.write(
            f'({setter} {v}'
            f' (not (= 0 (sys::peek buf ptr- :char))))'
            f' (incf ptr- 1)')
    elif f.base_type in ('octet', 'char'):
        s.write(
            f'({setter} {v} (sys::peek buf ptr- :char))'
            f' (incf ptr- 1)')
    elif _is_signed_int(f.base_type):
        _write_deserialize_bits_signed(s, v, NUM_BYTES[f.base_type])
        if NUM_BYTES[f.base_type] == 1:
            s.write(
                f'(if (> {v} 127)'
                f' ({setter} {v} (- {v} 256)))')
    elif _is_unsigned_int(f.base_type):
        _write_deserialize_bits(s, v, NUM_BYTES[f.base_type])
    else:
        raise ValueError(f'{f.base_type} unknown')


def _write_deserialize_field(s, f):
    var = f'_{f.name}'
    s.write(f';; {f.type_str} {var}')
    if f.is_array:
        if f.is_builtin:
            if f.base_type in ('uint8', 'octet', 'char'):
                if f.array_len:
                    s.write(
                        f'(setq {var}'
                        f' (make-array {f.array_len}'
                        f' :element-type :char))')
                    s.write(
                        f'(replace {var} buf :start2 ptr-)'
                        f' (incf ptr- {f.array_len})')
                else:
                    s.write(
                        '(let ((n (sys::peek buf ptr- :integer)))'
                        ' (incf ptr- 4)')
                    s.write(
                        f'  (setq {var}'
                        f' (make-array n :element-type :char))')
                    s.write(
                        f'  (replace {var} buf :start2 ptr-)'
                        f' (incf ptr- n))')
            elif f.array_len:
                s.write(f'(dotimes (i (length {var}))')
                var = f'(elt {var} i)'
            else:
                bt = f.base_type
                if (_is_float(bt) or _is_integer(bt)
                        or _is_string(bt) or _is_bool(bt)):
                    s.write('(let (n)')
                    with Indent(s):
                        s.write(
                            '(setq n (sys::peek buf ptr- :integer))'
                            ' (incf ptr- 4)')
                        if _is_string(bt) or _is_bool(bt):
                            s.write(f'(setq {var} (make-list n))')
                        else:
                            lt = _lisp_type(bt, True)
                            s.write(
                                f'(setq {var}'
                                f' (instantiate {lt}-vector n))')
                        s.write('(dotimes (i n)')
                        var = f'(elt {var} i)'
                else:
                    _write_deserialize_length(s, f, var, True)
                    var = 'elem-'
                    with Indent(s):
                        s.write(f'(dolist ({var} _{f.name})')
        else:
            # array of non-builtin
            if f.array_len:
                s.write(f'(dotimes (i {f.array_len})')
                var = f'(elt _{f.name} i)'
            else:
                _write_deserialize_length(s, f, var, True)
                var = 'elem-'
                with Indent(s):
                    s.write(f'(dolist ({var} _{f.name})')

    if f.is_array and f.base_type in ('uint8', 'octet', 'char'):
        pass
    elif f.is_builtin:
        with Indent(s):
            _write_deserialize_builtin(s, f, var)
    else:
        with Indent(s):
            s.write(
                f'(send {var} :deserialize buf ptr-)'
                f' (incf ptr- (send {var} :serialization-length))')

    if f.is_array and f.base_type not in ('uint8', 'octet', 'char'):
        with Indent(s):
            if f.array_len:
                s.write(')')
            else:
                s.write('))')


def _write_deserialize(s, fields):
    """Write :deserialize method.

    Note: does NOT close the (defmethod ...) form.
    The closing ) is written by _write_deserialize_cdr().
    """
    with Indent(s):
        s.write('(:deserialize')
        with Indent(s, inc=1):
            s.write('(buf &optional (ptr- 0))')
            for f in fields:
                _write_deserialize_field(s, f)
            s.write(';;')
            s.write('self)')


# -- Serialization length --

def _write_builtin_length(s, f):
    bt = f.base_type
    if bt in ('int8', 'uint8', 'octet', 'char', 'boolean'):
        s.write('1')
    elif bt in ('int16', 'uint16', 'wchar', 'short', 'unsigned short'):
        s.write('2')
    elif bt in ('int32', 'uint32', 'float', 'long', 'unsigned long'):
        s.write('4')
    elif bt in ('int64', 'uint64', 'double', 'long double',
                'long long', 'unsigned long long'):
        s.write('8')
    elif _is_string(bt):
        s.write(f'4 (length _{f.name})')
    else:
        raise ValueError(f'Unknown: {bt}')


def _write_serialization_length(s, fields):
    """Write :serialization-length method."""
    with Indent(s):
        s.write('(:serialization-length')
        with Indent(s, inc=1):
            s.write('()')
            s.write('(+')
            with Indent(s, 1):
                if not fields:
                    s.write('0')
                for f in fields:
                    s.write(f';; {f.type_str} _{f.name}')
                    if f.is_array:
                        if (f.is_builtin
                                and not _is_string(f.base_type)):
                            s.write('(* ')
                        else:
                            s.write('(apply #\'+ ')
                        s.block_next_indent()

                        if f.is_builtin:
                            if not f.array_len:
                                if _is_string(f.base_type):
                                    s.write(
                                        f'(mapcar #\'(lambda (x)'
                                        f' (+ 4 (length x)))'
                                        f' _{f.name})) 4')
                                else:
                                    _write_builtin_length(s, f)
                                    s.write(
                                        f'(length _{f.name})) 4',
                                        newline=False)
                            else:
                                _write_builtin_length(s, f)
                                s.write(
                                    f'{f.array_len})',
                                    newline=False)
                        else:
                            if f.array_len:
                                s.write(
                                    f'(send-all _{f.name}'
                                    f' :serialization-length))')
                            else:
                                s.write(
                                    f'(send-all _{f.name}'
                                    f' :serialization-length)) 4')
                    else:
                        if f.is_builtin:
                            _write_builtin_length(s, f)
                        else:
                            s.write(
                                f'(send _{f.name}'
                                f' :serialization-length)')

                s.write('))')


# ============================================================
# CDR serialization helpers
# ============================================================

def _cdr_alignment(f):
    """Return the CDR alignment value for a field's element type."""
    if not f.is_builtin:
        return 4  # struct alignment
    bt = f.base_type
    if bt in ('int8', 'uint8', 'octet', 'char', 'boolean'):
        return 1
    elif bt in ('int16', 'uint16', 'wchar', 'short', 'unsigned short'):
        return 2
    elif bt in ('int32', 'uint32', 'float', 'long', 'unsigned long',
                'string'):
        return 4  # string length field is uint32
    elif bt in ('int64', 'uint64', 'double', 'long double',
                'long long', 'unsigned long long'):
        return 8
    return 1


def _write_cdr_align_serialize(s, align, guard=None):
    """Write CDR alignment padding for serialize (stream-based).

    When GUARD (a lisp predicate string) is given, the padding is only
    emitted when GUARD is true. This is used for the element alignment of
    dynamic arrays: CDR does not insert element-alignment padding after the
    length of an EMPTY sequence, so the padding must be skipped when the
    array has no elements (otherwise nested messages get corrupted).
    """
    if align <= 1:
        return
    if guard:
        s.write(f'(when {guard}')
    # Subtract 4 for the CDR header that was written at the start
    s.write(
        f'(let ((pad (logand (- {align}'
        f' (logand (- (stream-count s) 4) {align - 1}))'
        f' {align - 1})))')
    s.write('  (dotimes (_ pad) (write-byte 0 s)))' + (')' if guard else ''))


def _write_cdr_align_deserialize(s, align, guard=None):
    """Write CDR alignment for deserialize (advance ptr-).

    GUARD works as in _write_cdr_align_serialize: the alignment is only
    applied when GUARD is true (used to skip element alignment for empty
    dynamic arrays, matching the serialize side and upstream rmw).
    """
    if align <= 1:
        return
    # Alignment is relative to CDR payload start (after 4-byte header)
    line = (
        f'(setq ptr- (+ 4 (logand (+ (- ptr- 4) {align - 1})'
        f' (lognot {align - 1}))))')
    if guard:
        s.write(f'(when {guard} {line})')
    else:
        s.write(line)


def _write_serialize_cdr_builtin(s, f, v):
    """Write CDR serialization for a single builtin value."""
    if _is_string(f.base_type):
        # CDR string: length includes null terminator
        s.write(f'(write-long (1+ (length {v})) s)')
        s.write(f'(princ {v} s)')
        s.write('(write-byte 0 s)')
    elif f.base_type == 'float':
        s.write(
            f'(sys::poke {v} (send s :buffer)'
            f' (send s :count) :float)'
            f' (incf (stream-count s) 4)')
    elif f.base_type in ('double', 'long double'):
        s.write(
            f'(sys::poke {v} (send s :buffer)'
            f' (send s :count) :double)'
            f' (incf (stream-count s) 8)')
    elif _is_bool(f.base_type):
        # CDR boolean: true=0x01 (not 0xFF)
        s.write(f'(if {v} (write-byte 1 s) (write-byte 0 s))')
    elif f.base_type in ('octet', 'char'):
        s.write(f'(write-byte {v} s)')
    elif _is_signed_int(f.base_type) or _is_unsigned_int(f.base_type):
        _write_serialize_bits(s, v, NUM_BYTES[f.base_type])
    else:
        raise ValueError(f'Unknown type: {f.base_type}')


def _write_serialize_cdr_field(s, f):
    """Write CDR serialization for a single field."""
    s.write(f';; {f.type_str} _{f.name}')
    slot = f'_{f.name}'
    var = slot

    if f.is_array and f.base_type in ('uint8', 'octet', 'char'):
        # byte array: length prefix for dynamic, then raw bytes
        align = 4 if not f.array_len else _cdr_alignment(f)
        _write_cdr_align_serialize(s, align)
        if not f.array_len:
            s.write(f'(write-long (length {slot}) s)')
        s.write(f'(princ {slot} s)')
    elif f.is_array and _is_string(f.base_type):
        _write_cdr_align_serialize(s, 4)
        s.write(f'(write-long (length {slot}) s)')
        s.write(f'(dolist (elem {slot})')
        with Indent(s):
            _write_cdr_align_serialize(s, 4)
            _write_serialize_cdr_builtin(s, f, 'elem')
        s.write('  )')
    elif f.is_array:
        if not f.array_len:
            _write_cdr_align_serialize(s, 4)
            s.write(f'(write-long (length {slot}) s)')
        if f.is_builtin:
            align = _cdr_alignment(f)
            # Skip element alignment for empty dynamic arrays (see
            # _write_cdr_align_serialize); fixed-length arrays always align.
            _write_cdr_align_serialize(
                s, align,
                guard=(None if f.array_len else f'(> (length {slot}) 0)'))
            if f.array_len:
                s.write(f'(dotimes (i {f.array_len})')
            else:
                s.write(f'(dotimes (i (length {var}))')
            var = f'(elt {var} i)'
            with Indent(s):
                _write_serialize_cdr_builtin(s, f, var)
            s.write('  )')
        else:
            s.write(f'(dolist (elem {slot})')
            with Indent(s):
                _write_cdr_align_serialize(s, 4)
                s.write(f'(send elem :serialize-cdr s)')
            s.write('  )')
    elif f.is_builtin:
        align = _cdr_alignment(f)
        _write_cdr_align_serialize(s, align)
        _write_serialize_cdr_builtin(s, f, var)
    else:
        _write_cdr_align_serialize(s, 4)
        s.write(f'(send {slot} :serialize-cdr s)')


def _write_serialize_cdr(s, fields):
    """Write :serialize-cdr method."""
    with Indent(s):
        s.write('(:serialize-cdr')
        with Indent(s, inc=1):
            s.write('(&optional strm)')
            s.write('(let ((s (if strm strm')
            s.write(
                '           (make-string-output-stream'
                ' (send self :serialization-length-cdr)))))')
            with Indent(s):
                # CDR header only at top-level (when strm is nil)
                s.write(';; CDR Header (4 bytes) - only at top level')
                s.write('(unless strm')
                s.write('  (write-byte 0 s)')
                s.write('  (write-byte 1 s)')
                s.write('  (write-byte 0 s)')
                s.write('  (write-byte 0 s))')
                for f in fields:
                    _write_serialize_cdr_field(s, f)
                s.write(';;')
                s.write(
                    '(if (null strm)'
                    ' (get-output-stream-string s))))')


def _write_deserialize_cdr_builtin(s, f, v):
    """Write CDR deserialization for a single builtin value."""
    setter = 'setf' if v[0] == '(' else 'setq'
    if _is_string(f.base_type):
        # CDR string: length includes null, read len-1 chars, skip null
        s.write(
            f'(let ((n (sys::peek buf ptr- :integer))) (incf ptr- 4)')
        s.write(
            f'  ({setter} {v}'
            f' (subseq buf ptr- (+ ptr- (1- n)))) (incf ptr- n))')
    elif f.base_type == 'float':
        s.write(
            f'({setter} {v} (sys::peek buf ptr- :float))'
            f' (incf ptr- 4)')
    elif f.base_type in ('double', 'long double'):
        s.write(
            f'({setter} {v} (sys::peek buf ptr- :double))'
            f' (incf ptr- 8)')
    elif _is_bool(f.base_type):
        # CDR boolean: 0x01 = true
        s.write(
            f'({setter} {v}'
            f' (not (= 0 (sys::peek buf ptr- :char))))'
            f' (incf ptr- 1)')
    elif f.base_type in ('octet', 'char'):
        s.write(
            f'({setter} {v} (sys::peek buf ptr- :char))'
            f' (incf ptr- 1)')
    elif _is_signed_int(f.base_type):
        _write_deserialize_bits_signed(s, v, NUM_BYTES[f.base_type])
        if NUM_BYTES[f.base_type] == 1:
            s.write(
                f'(if (> {v} 127)'
                f' ({setter} {v} (- {v} 256)))')
    elif _is_unsigned_int(f.base_type):
        _write_deserialize_bits(s, v, NUM_BYTES[f.base_type])
    else:
        raise ValueError(f'{f.base_type} unknown')


def _write_deserialize_cdr_field(s, f):
    """Write CDR deserialization for a single field."""
    var = f'_{f.name}'
    s.write(f';; {f.type_str} {var}')

    if f.is_array:
        if f.is_builtin:
            if f.base_type in ('uint8', 'octet', 'char'):
                if f.array_len:
                    _write_cdr_align_deserialize(
                        s, _cdr_alignment(f))
                    s.write(
                        f'(setq {var}'
                        f' (make-array {f.array_len}'
                        f' :element-type :char))')
                    s.write(
                        f'(replace {var} buf :start2 ptr-)'
                        f' (incf ptr- {f.array_len})')
                else:
                    _write_cdr_align_deserialize(s, 4)
                    s.write(
                        '(let ((n (sys::peek buf ptr- :integer)))'
                        ' (incf ptr- 4)')
                    s.write(
                        f'  (setq {var}'
                        f' (make-array n :element-type :char))')
                    s.write(
                        f'  (replace {var} buf :start2 ptr-)'
                        f' (incf ptr- n))')
            elif _is_string(f.base_type):
                _write_cdr_align_deserialize(s, 4)
                s.write(
                    '(let ((n (sys::peek buf ptr- :integer)))'
                    ' (incf ptr- 4)')
                s.write(f'  (setq {var} (make-list n))')
                s.write('  (dotimes (i n)')
                with Indent(s, inc=2):
                    _write_cdr_align_deserialize(s, 4)
                    _write_deserialize_cdr_builtin(
                        s, f, f'(elt {var} i)')
                s.write('  ))')
            elif f.array_len:
                _write_cdr_align_deserialize(
                    s, _cdr_alignment(f))
                s.write(f'(dotimes (i (length {var}))')
                with Indent(s):
                    _write_deserialize_cdr_builtin(
                        s, f, f'(elt {var} i)')
                s.write('  )')
            else:
                # dynamic array of primitives
                _write_cdr_align_deserialize(s, 4)
                bt = f.base_type
                s.write('(let (n)')
                with Indent(s):
                    s.write(
                        '(setq n (sys::peek buf ptr- :integer))'
                        ' (incf ptr- 4)')
                    if _is_bool(bt):
                        s.write(f'(setq {var} (make-list n))')
                    else:
                        lt = _lisp_type(bt, True)
                        s.write(
                            f'(setq {var}'
                            f' (instantiate {lt}-vector n))')
                    _write_cdr_align_deserialize(
                        s, _cdr_alignment(f), guard='(> n 0)')
                    s.write('(dotimes (i n)')
                    with Indent(s):
                        _write_deserialize_cdr_builtin(
                            s, f, f'(elt {var} i)')
                    s.write('))')
        else:
            # array of non-builtin (messages)
            if f.array_len:
                s.write(f'(dotimes (i {f.array_len})')
                with Indent(s):
                    _write_cdr_align_deserialize(s, 4)
                    s.write(
                        f'(setq ptr-'
                        f' (send (elt _{f.name} i)'
                        f' :deserialize-cdr buf ptr-))')
                s.write('  )')
            else:
                _write_cdr_align_deserialize(s, 4)
                ft = _field_type(f)
                s.write('(let (n)')
                with Indent(s):
                    s.write(
                        '(setq n (sys::peek buf ptr- :integer))'
                        ' (incf ptr- 4)')
                    s.write(
                        f'(setq {var} (let (r) (dotimes (i n)'
                        f' (push (instance {ft} :init) r)) r))')
                    s.write(f'(dolist (elem- {var})')
                    with Indent(s):
                        _write_cdr_align_deserialize(s, 4)
                        s.write(
                            '(setq ptr-'
                            ' (send elem- :deserialize-cdr buf ptr-))')
                    s.write('))')
    elif f.is_builtin:
        align = _cdr_alignment(f)
        _write_cdr_align_deserialize(s, align)
        _write_deserialize_cdr_builtin(s, f, var)
    else:
        _write_cdr_align_deserialize(s, 4)
        s.write(
            f'(setq ptr-'
            f' (send {var} :deserialize-cdr buf ptr-))')


def _write_deserialize_cdr(s, fields):
    """Write :deserialize-cdr method.

    Returns ptr- (final position in buffer) so callers can track exact
    byte consumption.  When called at top level (ptr-=0), skips the
    4-byte CDR header automatically.
    """
    with Indent(s):
        s.write('(:deserialize-cdr')
        with Indent(s, inc=1):
            s.write('(buf &optional (ptr- 0))')
            # Skip CDR header only at top-level (ptr- = 0)
            s.write(';; skip CDR header 4 bytes (only at top level)')
            s.write('(when (= ptr- 0) (incf ptr- 4))')
            for f in fields:
                _write_deserialize_cdr_field(s, f)
            s.write(';;')
            s.write('ptr-)')
        s.write(')')
        s.newline()


def _write_cdr_builtin_length(s, f):
    """Write the CDR byte size contribution for a builtin field."""
    bt = f.base_type
    if bt in ('int8', 'uint8', 'octet', 'char', 'boolean'):
        s.write('1')
    elif bt in ('int16', 'uint16', 'wchar', 'short', 'unsigned short'):
        s.write('2')
    elif bt in ('int32', 'uint32', 'float', 'long', 'unsigned long'):
        s.write('4')
    elif bt in ('int64', 'uint64', 'double', 'long double',
                'long long', 'unsigned long long'):
        s.write('8')
    elif _is_string(bt):
        # CDR string: 4-byte length + chars + 1 null byte
        s.write(f'4 (length _{f.name}) 1')
    else:
        raise ValueError(f'Unknown: {bt}')


def _write_serialization_length_cdr(s, fields):
    """Write :serialization-length-cdr method.

    CDR serialization length includes:
    - 4-byte CDR header
    - Alignment padding (worst case estimated)
    - Field data
    """
    with Indent(s):
        s.write('(:serialization-length-cdr')
        with Indent(s, inc=1):
            s.write('()')
            s.write('(+')
            with Indent(s, 1):
                # CDR header is always 4 bytes
                s.write('4  ;; CDR header')
                if not fields:
                    s.write('0')
                for f in fields:
                    s.write(f';; {f.type_str} _{f.name}')
                    align = _cdr_alignment(f)
                    if align > 1:
                        s.write(
                            f'{align - 1}  ;; max alignment padding')

                    if f.is_array:
                        if (f.is_builtin
                                and not _is_string(f.base_type)):
                            s.write('(* ')
                        else:
                            s.write('(apply #\'+ ')
                        s.block_next_indent()

                        if f.is_builtin:
                            if not f.array_len:
                                if _is_string(f.base_type):
                                    # Each string: 4 len + chars + 1
                                    # null + up to 3 align padding
                                    s.write(
                                        f'(mapcar #\'(lambda (x)'
                                        f' (+ 4 (length x) 1 3))'
                                        f' _{f.name})) 4')
                                else:
                                    _write_cdr_builtin_length(s, f)
                                    s.write(
                                        f'(length _{f.name})) 4',
                                        newline=False)
                            else:
                                _write_cdr_builtin_length(s, f)
                                s.write(
                                    f'{f.array_len})',
                                    newline=False)
                        else:
                            # Subtract 4 per element for CDR headers of
                            # nested messages (no sub-headers in DDS CDR).
                            # Use mapcar to compute (len - 4) per element
                            # so that apply #'+ receives a proper list
                            # (also handles empty sequences correctly).
                            if f.array_len:
                                s.write(
                                    f'(mapcar'
                                    f' #\'(lambda (x)'
                                    f' (- (send x'
                                    f' :serialization-length-cdr) 4))'
                                    f' _{f.name}))')
                            else:
                                s.write(
                                    f'(mapcar'
                                    f' #\'(lambda (x)'
                                    f' (- (send x'
                                    f' :serialization-length-cdr) 4))'
                                    f' _{f.name})) 4')
                    else:
                        if f.is_builtin:
                            _write_cdr_builtin_length(s, f)
                        else:
                            # Subtract 4 for CDR header: nested messages
                            # don't have their own CDR header in DDS CDR
                            s.write(
                                f'(- (send _{f.name}'
                                f' :serialization-length-cdr) 4)')

                s.write('))')


def _write_ros_datatype(s, pkg, name, subfolder):
    """Write :datatype- property (ROS 2 format)."""
    s.write(
        f'(setf (get {pkg}::{name} :datatype-)'
        f' "{pkg}/{subfolder}/{name}")')


# ============================================================
# Top-level message / service generators
# ============================================================

def _write_msg(s, pkg, name, fields, constants, subfolder='msg'):
    """Generate complete .l content for a message."""
    _write_begin(s, pkg, name)
    _write_include(s, pkg, fields)
    _write_constants(s, pkg, name, constants)
    _write_defclass(s, pkg, name, fields)
    _write_defmethod(s, pkg, name, fields)
    _write_accessors(s, fields)
    _write_serialization_length(s, fields)
    _write_serialize(s, fields)
    _write_deserialize(s, fields)
    _write_serialization_length_cdr(s, fields)
    _write_serialize_cdr(s, fields)
    _write_deserialize_cdr(s, fields)
    _write_ros_datatype(s, pkg, name, subfolder)


def _write_srv_component(s, pkg, comp_name, fields, constants):
    """Generate the request/response component of a service."""
    _write_constants(s, pkg, comp_name, constants)
    _write_defclass(s, pkg, comp_name, fields)
    _write_defmethod(s, pkg, comp_name, fields)
    _write_accessors(s, fields)
    _write_serialization_length(s, fields)
    _write_serialize(s, fields)
    _write_deserialize(s, fields)
    _write_serialization_length_cdr(s, fields)
    _write_serialize_cdr(s, fields)
    _write_deserialize_cdr(s, fields)


def _write_service_specific_methods(s, pkg, srv_name, req_name, res_name):
    """Generate service-level class and linkage."""
    s.write(f'(defclass {pkg}::{srv_name}')
    with Indent(s):
        s.write(':super ros::object')
        s.write(':slots ())')
    s.newline()
    _write_ros_datatype(s, pkg, srv_name, 'srv')
    s.write(
        f'(setf (get {pkg}::{srv_name} :request)'
        f' {pkg}::{req_name})')
    s.write(
        f'(setf (get {pkg}::{srv_name} :response)'
        f' {pkg}::{res_name})')
    s.newline()
    s.write(f'(defmethod {pkg}::{req_name}')
    s.write(
        f'  (:response () (instance {pkg}::{res_name} :init)))')
    s.newline()
    _write_ros_datatype(s, pkg, req_name, 'srv')
    s.newline()
    _write_ros_datatype(s, pkg, res_name, 'srv')
    s.newline()
    s.write('\n')


def _write_srv(s, pkg, srv_name, request_msg, response_msg):
    """Generate complete .l content for a service."""
    req_name = f'{srv_name}Request'
    res_name = f'{srv_name}Response'

    req_fields = [FieldInfo(m) for m in request_msg.structure.members]
    res_fields = [FieldInfo(m) for m in response_msg.structure.members]

    _write_begin(s, pkg, srv_name, is_service=True)
    _write_include(s, pkg, req_fields)
    _write_include(s, pkg, res_fields)

    _write_srv_component(
        s, pkg, req_name, req_fields, request_msg.constants)
    _write_srv_component(
        s, pkg, res_name, res_fields, response_msg.constants)
    _write_service_specific_methods(s, pkg, srv_name, req_name, res_name)


def _write_action(s, pkg, action):
    """Generate complete .l content for an action.

    An action file contains:
    - Goal, Result, Feedback message classes
    - SendGoal, GetResult service classes
    - FeedbackMessage message class
    - Action meta-class with property links
    """
    act_name = action.namespaced_type.name

    # Collect all fields for dependency includes
    goal_fields = [FieldInfo(m) for m in action.goal.structure.members]
    result_fields = [FieldInfo(m) for m in action.result.structure.members]
    feedback_fields = [FieldInfo(m) for m in action.feedback.structure.members]

    sg_req_fields = [
        FieldInfo(m) for m in
        action.send_goal_service.request_message.structure.members]
    sg_res_fields = [
        FieldInfo(m) for m in
        action.send_goal_service.response_message.structure.members]
    gr_req_fields = [
        FieldInfo(m) for m in
        action.get_result_service.request_message.structure.members]
    gr_res_fields = [
        FieldInfo(m) for m in
        action.get_result_service.response_message.structure.members]
    fb_msg_fields = [
        FieldInfo(m) for m in action.feedback_message.structure.members]

    all_fields = (goal_fields + result_fields + feedback_fields +
                  sg_req_fields + sg_res_fields +
                  gr_req_fields + gr_res_fields + fb_msg_fields)

    # File header - create packages for all sub-types
    s.write(';; Auto-generated. Do not edit!\n\n', newline=False)

    # Main action package
    s.write(f'(when (boundp \'{pkg}::{act_name})')
    s.write(f'  (if (not (find-package "{pkg.upper()}"))')
    s.write(f'    (make-package "{pkg.upper()}"))')
    s.write(f'  (shadow \'{act_name} (find-package "{pkg.upper()}")))')

    sub_names = [
        act_name,
        f'{act_name}_Goal', f'{act_name}_Result', f'{act_name}_Feedback',
        f'{act_name}_SendGoal',
        f'{act_name}_SendGoalRequest', f'{act_name}_SendGoalResponse',
        f'{act_name}_GetResult',
        f'{act_name}_GetResultRequest', f'{act_name}_GetResultResponse',
        f'{act_name}_FeedbackMessage',
    ]
    for sub in sub_names:
        s.write(f'(unless (find-package "{pkg.upper()}::{sub.upper()}")')
        s.write(f'  (make-package "{pkg.upper()}::{sub.upper()}"))')

    s.write('')
    s.write('(in-package "ROS")')
    s.newline()

    # Dependency includes
    _write_include(s, pkg, all_fields)

    # Goal message
    goal_name = f'{act_name}_Goal'
    _write_msg(s, pkg, goal_name, goal_fields,
               action.goal.constants, subfolder='action')

    # Result message
    result_name = f'{act_name}_Result'
    _write_msg(s, pkg, result_name, result_fields,
               action.result.constants, subfolder='action')

    # Feedback message
    feedback_name = f'{act_name}_Feedback'
    _write_msg(s, pkg, feedback_name, feedback_fields,
               action.feedback.constants, subfolder='action')

    # SendGoal service
    sg_name = f'{act_name}_SendGoal'
    sg_req_name = f'{sg_name}Request'
    sg_res_name = f'{sg_name}Response'
    _write_srv_component(
        s, pkg, sg_req_name, sg_req_fields,
        action.send_goal_service.request_message.constants)
    _write_srv_component(
        s, pkg, sg_res_name, sg_res_fields,
        action.send_goal_service.response_message.constants)
    _write_service_specific_methods(
        s, pkg, sg_name, sg_req_name, sg_res_name)

    # GetResult service
    gr_name = f'{act_name}_GetResult'
    gr_req_name = f'{gr_name}Request'
    gr_res_name = f'{gr_name}Response'
    _write_srv_component(
        s, pkg, gr_req_name, gr_req_fields,
        action.get_result_service.request_message.constants)
    _write_srv_component(
        s, pkg, gr_res_name, gr_res_fields,
        action.get_result_service.response_message.constants)
    _write_service_specific_methods(
        s, pkg, gr_name, gr_req_name, gr_res_name)

    # FeedbackMessage
    fb_msg_name = f'{act_name}_FeedbackMessage'
    _write_msg(s, pkg, fb_msg_name, fb_msg_fields,
               action.feedback_message.constants, subfolder='action')

    # Action meta-class
    s.write(f'(defclass {pkg}::{act_name}')
    with Indent(s):
        s.write(':super ros::object')
        s.write(':slots ())')
    s.newline()
    _write_ros_datatype(s, pkg, act_name, 'action')
    s.write(
        f'(setf (get {pkg}::{act_name} :goal)'
        f' {pkg}::{goal_name})')
    s.write(
        f'(setf (get {pkg}::{act_name} :result)'
        f' {pkg}::{result_name})')
    s.write(
        f'(setf (get {pkg}::{act_name} :feedback)'
        f' {pkg}::{feedback_name})')
    s.write(
        f'(setf (get {pkg}::{act_name} :send-goal-service)'
        f' {pkg}::{sg_name})')
    s.write(
        f'(setf (get {pkg}::{act_name} :get-result-service)'
        f' {pkg}::{gr_name})')
    s.write(
        f'(setf (get {pkg}::{act_name} :feedback-message)'
        f' {pkg}::{fb_msg_name})')
    s.newline()

    # ROS 1 backward compatibility aliases
    # In ROS 1: FibonacciAction, FibonacciGoal, FibonacciActionGoal, etc.
    # In ROS 2: Fibonacci, Fibonacci_Goal, Fibonacci_SendGoalRequest, etc.
    s.write(f';; ROS 1 backward compatibility aliases')
    s.write(f'(setq {pkg}::{act_name}Action {pkg}::{act_name})')
    s.write(f'(setq {pkg}::{act_name}Goal {pkg}::{goal_name})')
    s.write(f'(setq {pkg}::{act_name}Result {pkg}::{result_name})')
    s.write(f'(setq {pkg}::{act_name}Feedback {pkg}::{feedback_name})')
    s.write(
        f'(setq {pkg}::{act_name}ActionGoal'
        f' {pkg}::{sg_req_name})')
    s.write(
        f'(setq {pkg}::{act_name}ActionResult'
        f' {pkg}::{gr_res_name})')
    s.write(
        f'(setq {pkg}::{act_name}ActionFeedback'
        f' {pkg}::{fb_msg_name})')
    s.newline()
    s.write('\n')


# ============================================================
# Main entry point
# ============================================================

def generate_eus(generator_arguments_file):
    """
    Generate EusLisp .l files from IDL.

    :param generator_arguments_file: path to the JSON arguments file
    :returns: list of generated file paths
    """
    args = read_generator_arguments(generator_arguments_file)
    package_name = args['package_name']
    output_dir = pathlib.Path(args['output_dir'])

    generated_files = []

    for idl_tuple in args.get('idl_tuples', []):
        idl_parts = idl_tuple.rsplit(':', 1)
        assert len(idl_parts) == 2
        locator = IdlLocator(*idl_parts)
        idl_file = parse_idl_file(locator)

        # Collect action-derived service names to skip in Service loop
        action_service_names = set()
        for action in idl_file.content.get_elements_of_type(Action):
            act_name = action.namespaced_type.name
            action_service_names.add(f'{act_name}_SendGoal')
            action_service_names.add(f'{act_name}_GetResult')

        # Collect service/action message names to skip in message iteration
        service_msg_names = set()
        for service in idl_file.content.get_elements_of_type(Service):
            srv_name = service.namespaced_type.name
            if srv_name in action_service_names:
                continue
            service_msg_names.add(f'{srv_name}_Request')
            service_msg_names.add(f'{srv_name}_Response')
            service_msg_names.add(f'{srv_name}_Event')

            out_file = output_dir / 'srv' / f'{srv_name}.l'
            os.makedirs(out_file.parent, exist_ok=True)

            io = StringIO()
            s = IndentedWriter(io)
            _write_srv(
                s, package_name, srv_name,
                service.request_message, service.response_message)

            with open(out_file, 'w') as fh:
                fh.write(io.getvalue() + '\n')
            io.close()
            generated_files.append(str(out_file))

        # Collect action-derived message/service names to skip
        action_msg_names = set()
        for action in idl_file.content.get_elements_of_type(Action):
            act_name = action.namespaced_type.name
            # Action generates these sub-types automatically
            action_msg_names.add(f'{act_name}_Goal')
            action_msg_names.add(f'{act_name}_Result')
            action_msg_names.add(f'{act_name}_Feedback')
            action_msg_names.add(f'{act_name}_SendGoal_Request')
            action_msg_names.add(f'{act_name}_SendGoal_Response')
            action_msg_names.add(f'{act_name}_SendGoal_Event')
            action_msg_names.add(f'{act_name}_GetResult_Request')
            action_msg_names.add(f'{act_name}_GetResult_Response')
            action_msg_names.add(f'{act_name}_GetResult_Event')
            action_msg_names.add(f'{act_name}_FeedbackMessage')

            out_file = output_dir / 'action' / f'{act_name}.l'
            os.makedirs(out_file.parent, exist_ok=True)

            io = StringIO()
            s = IndentedWriter(io)
            _write_action(s, package_name, action)

            with open(out_file, 'w') as fh:
                fh.write(io.getvalue() + '\n')
            io.close()
            generated_files.append(str(out_file))

        skip_names = service_msg_names | action_msg_names

        for message in idl_file.content.get_elements_of_type(Message):
            msg_name = message.structure.namespaced_type.name
            if msg_name in skip_names:
                continue

            fields = [FieldInfo(m) for m in message.structure.members]
            # Determine subfolder from namespaces (msg/srv/action)
            ns = message.structure.namespaced_type.namespaces
            subfolder = ns[1] if len(ns) > 1 else 'msg'

            out_file = output_dir / subfolder / f'{msg_name}.l'
            os.makedirs(out_file.parent, exist_ok=True)

            io = StringIO()
            s = IndentedWriter(io)
            _write_msg(
                s, package_name, msg_name, fields,
                message.constants, subfolder)

            with open(out_file, 'w') as fh:
                fh.write(io.getvalue() + '\n')
            io.close()
            generated_files.append(str(out_file))

    return generated_files
