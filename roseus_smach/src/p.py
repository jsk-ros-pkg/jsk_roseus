import base64
import pickle

#print(base64.b64decode("gAJ9cQAu"))
#print(pickle.loads(base64.b64decode("gAJ9cQAu"))) # -> {}

objs = []
for msg in [True, False, None, 'hello', 10, 12.3, "Hello\nWorld", [1, 2, 3], [1, 12.3, "Hello", True, False], {0: "zero", "Hello": "World"} ]:
    obj = base64.b64encode(pickle.dumps(msg,0))
    print("({} . \"{}\")".format(msg, obj))
    objs.append(obj)
    #print(msg.decode('utf-8'))
    # print(base64.b64decode(msg))
    # print(pickle.loads(base64.b64decode(msg)))

for obj in objs:
    print("{} -> {}".format(base64.b64decode(obj), pickle.loads(base64.b64decode(obj))))
