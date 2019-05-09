#!/usr/bin/env python
"""Protobuf tutorial."""

import addressbook_pb2
person = addressbook_pb2.Person()
person.id = 1234
person.name = "John Doe"
person.email = "jdoe@example.com"
phone = person.phones.add()
phone.number = "555-4321"
phone.type = addressbook_pb2.Person.HOME

phone2 = person.phones.add()
phone2.number = "444-3322"
phone2.type = addressbook_pb2.Person.WORK
print person.IsInitialized()

print (person)
test_string = person.SerializeToString()
print test_string
person2 = addressbook_pb2.Person()
person2.ParseFromString(test_string)
print "new person:"
print(person2)
person.Clear()
print person.IsInitialized()
print person

