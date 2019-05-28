from flask import Flask, request
from flask_restful import Resource, Api

app = Flask(__name__)
api = Api(app)

todos = {}

def argParse(arg):
    listArg = arg.split("&")
    dictArg = {}
    for args in listArg:
        temp = args.split("=")
        dictArg[temp[0]] = temp[1]

    return dictArg

class HelloWorld(Resource):
    def get(self):
        return "oba"

class TodoSimple(Resource):
    def get(self, todo_id):
        args = argParse(todo_id)
        return args

    def put(self, todo_id):
        todos[todo_id] = request.form['data']
        return {todo_id: todos[todo_id]}

api.add_resource(TodoSimple, '/<string:todo_id>')
api.add_resource(HelloWorld, '/')

if __name__ == '__main__':
    app.run(host='0.0.0.0',debug=True)
    #app.run(debug=True)

