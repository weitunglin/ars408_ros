lis = [{"id":1, "name":"a"},{"id":2, "name":"b"},{"id":3, "name":"c"},{"id":4, "name":"d"}]

print(lis)

if(next((i for i in lis if i['id'] == 1), None) != None):
    lis.remove(next(i for i in lis if i['id'] == 1))


print(lis)