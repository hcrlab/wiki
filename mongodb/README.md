# MongoDB

## mongodb_store
Here are some tips for using [mongodb_store](http://wiki.ros.org/mongodb_store).

## PyMongo version
You will need to use pymongo version 2.3:
```
sudo pip install pymongo==2.3
```

### Launching
Launch file for using the local MongoDB daemon:
```xml
<launch>
  <param name="mongodb_use_daemon" value="true" />
  <param name="mongodb_host" value="localhost" />
  <param name="mongodb_port" value="27017" />
  <node name="message_store" pkg="mongodb_store" type="message_store_node.py" output="screen" /> 
</launch>
```

### Changing the database / collection
The constructors for `MessageStoreProxy` in Python and C++ have default values for the MongoDB database and collection.
Look at the source ([Python](https://github.com/strands-project/mongodb_store/blob/hydro-devel/mongodb_store/src/mongodb_store/message_store.py), [C++](https://github.com/strands-project/mongodb_store/blob/hydro-devel/mongodb_store/include/mongodb_store/message_store.h)) to see how to pass in different values.

### C++ update/insert error
If you are using the C++ API for `mongodb_store`, you might encounter an md5 error when updating a document.
This is due to a bug the `mongodb_store::MessageStoreProxy` copy constructor.
We have submitted a patch, so hopefully this will not be a problem in the future.
In the meantime, you can avoid making copies by allocating a `MessageStoreProxy` once in your main function and passing a pointer to it to helper classes:
```cpp
// main.cpp
mongodb_store::MessageStoreProxy db(nh, collection_name, db_name);
AwesomeServer server(&db);
```
