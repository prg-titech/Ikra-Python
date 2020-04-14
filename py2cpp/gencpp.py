# -*- coding: utf-8 -*-
# Generate C/C++/CUDA code From C/C++/CUDA AST

import enum
import six

FILE_NAME = "sanajeh_device_code"


class Type(enum.Enum):
    Builder = 0
    Block = 1
    Expr = 2
    Stmt = 4
    arguments = 2048
    Comment = 9999


INDENT = " " * 4


class BuildContext(object):
    def __init__(self, ctx, node):
        self.indent_level = ctx.indent_level + 1
        self.stack = ctx.stack + [node]

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        return False if exc_type else True

    def indent(self):
        return INDENT * self.indent_level

    def is_class_method(self):
        if len(self.stack) < 2:
            return False
        last = self.stack[-1]
        if last.__class__ != FunctionDef:
            return False
        cls = self.stack[-2]
        return cls.__class__ == ClassDef

    def in_class(self):
        for node in reversed(self.stack):
            if node.__class__ != ClassDef:
                continue
            return True
        return False

    @staticmethod
    def create():
        class _DummyContext(object):
            indent_level = -1
            stack = []

        ret = BuildContext(_DummyContext(), None)
        ret.stack = []
        return ret


class Base(object):
    _fields = []

    def __init__(self, tp):
        self.type = tp

    def buildCpp(self, ctx):
        """
        :type ctx: BuildContext
        """
        assert False

    def buildHpp(self, ctx):
        return ""


# AST = Base

# Ignored nodes which are not run on the device
class IgnoredNode(Base):
    def __init__(self, node):
        super(IgnoredNode, self).__init__(Type.Comment)
        self.node = node

    def buildCpp(self, ctx):
        return ""
        # return "// IGNORED AST NODE: {}".format(self.node.__class__.__name__)


class UnsupportedNode(Base):
    def __init__(self, node):
        super(UnsupportedNode, self).__init__(Type.Comment)
        self.node = node

    def buildCpp(self, ctx):
        return ""
        # return "// UNSUPPORTED AST NODE: {}".format(self.node.__class__.__name__)


class Module(Base):
    _fields = ["body", "classes"]

    def __init__(self, body, classes):
        super(Module, self).__init__(Type.Block)
        self.body = body
        self.classes = classes

    def buildCpp(self, ctx):
        include_expr = '#include "{}.h"\n\n'.format(FILE_NAME)
        allocator_declaration = "AllocatorHandle<AllocatorT>* allocator_handle;\n" \
                                "__device__ AllocatorT* device_allocator;\n\n"
        rstr = ""
        for x in self.body:
            xstr = x.buildCpp(ctx)
            if not xstr == "":
                rstr += xstr + "\n"
        return include_expr + allocator_declaration + rstr

    def buildHpp(self, ctx):
        # todo num objects
        precompile_expr = "#ifndef SANAJEH_DEVICE_CODE_H" \
                          "\n#define SANAJEH_DEVICE_CODE_H" \
                          "\n#define KNUMOBJECTS 64*64*64*64"
        include_expr = '\n\n#include <curand_kernel.h>\n#include "dynasoar.h"'
        rstr = ""
        for x in self.body:
            xstr = x.buildHpp(ctx)
            if not xstr == "":
                rstr += xstr + "\n"
        class_str = ','.join(self.classes)
        class_predefine = "\n\nclass " + class_str + ';'
        endif_expr = "\n#endif"
        temp_expr = "\n\nusing AllocatorT = SoaAllocator<" + "KNUMOBJECTS" + ", " + class_str + ">;\n"
        return precompile_expr + include_expr + class_predefine + temp_expr + rstr + endif_expr


class CodeStatement(Base):
    _fields = ["stmt"]

    def __init__(self, stmt):
        super(CodeStatement, self).__init__(Type.Stmt)
        self.stmt = stmt


class FunctionDef(CodeStatement):
    _fields = ["name", "args", "body", "returns"]

    def __init__(self, name, args, body, returns=None):
        super(FunctionDef, self).__init__(body)
        self.name = name
        self.args = args
        self.returns = returns
        self.body = self.stmt

    def buildCpp(self, ctx):
        with BuildContext(ctx, self) as new_ctx:
            body = [x.buildCpp(new_ctx) for x in self.stmt]
            while '' in body:
                body.remove('')
            if ctx.in_class():
                # __init__ special case
                if self.name == "__init__":
                    return "\n".join([
                        "\n{}__device__ {}::{}({}) {{".format(
                            ctx.indent(),
                            ctx.stack[-1].name,
                            ctx.stack[-1].name,
                            self.args.buildCpp(new_ctx),
                        ),
                        "\n".join(body),
                        ctx.indent() + "}",
                    ])
                return "\n".join([
                    "\n{}__device__ {} {}::{}({}) {{".format(
                        ctx.indent(),
                        self.rtype(ctx),
                        ctx.stack[-1].name,
                        self.name,
                        self.args.buildCpp(new_ctx),
                    ),
                    "\n".join(body),
                    ctx.indent() + "}",
                ])
            else:
                return "\n".join([
                    # todo maybe global
                    "\n{}__device__ {} {}({}) {{".format(
                        ctx.indent(),
                        self.rtype(ctx),
                        self.name,
                        self.args.buildCpp(new_ctx),
                    ),
                    "\n".join(body),
                    ctx.indent() + "}",
                ])

    def buildHpp(self, ctx):
        with BuildContext(ctx, self) as new_ctx:
            body = [x.buildHpp(new_ctx) for x in self.stmt]
            while '' in body:
                body.remove('')
            # __init__ special case
            if self.name == "__init__" and ctx.in_class():
                return "\n" + "\n".join([
                    "{}__device__ {}({});".format(
                        ctx.indent(),
                        ctx.stack[-1].name,
                        self.args.buildHpp(new_ctx),
                    )
                ])
            return "\n".join([
                "{}__device__ {} {}({});".format(
                    ctx.indent(),
                    self.rtype(ctx),
                    self.name,
                    self.args.buildHpp(new_ctx),
                )
            ])

    def rtype(self, ctx):
        if self.returns:
            rtype = self.returns.buildCpp(ctx)
            return CppTypeRegistry.detect(rtype, rettype=True)
        else:
            return "void"


class ClassDef(CodeStatement):
    _fields = ["name", "bases", "body", "fields"]

    def __init__(self, name, bases, body, fields, **kwargs):
        super(ClassDef, self).__init__(body)
        self.name = name
        self.bases = bases
        self.keywords = kwargs.get("keywords", [])
        self.fields = fields

    # todo without class block
    def buildCpp(self, ctx):
        with BuildContext(ctx, self) as new_ctx:
            new_ctx.indent_level -= 1
            body = [x.buildCpp(new_ctx) for x in self.stmt if type(x) is FunctionDef]
            while '' in body:
                body.remove('')
            return "\n".join(body)

    def buildHpp(self, ctx):
        with BuildContext(ctx, self) as new_ctx:
            body = [x.buildHpp(new_ctx) for x in self.stmt]
            while '' in body:
                body.remove('')
            field_types = []
            field_templates = []
            i = 0
            for field in self.fields:
                field_types.append(self.fields[field])
                field_templates.append(INDENT + "Field<{}, {}> {};".format(self.name,
                                                                           i,
                                                                           field)
                                       )
                i += 1
            field_predeclaration = new_ctx.indent() \
                                   + "public:\n" \
                                   + new_ctx.indent() \
                                   + INDENT \
                                   + "declare_field_types({}, {})\n".format(self.name, ", ".join(field_types)) \
                                   + new_ctx.indent() \
                                   + "private:\n" \
                                   + new_ctx.indent() \
                                   + ("\n" + new_ctx.indent()).join(field_templates)
            return "\n".join([
                "\n{}class {}{} {{".format(
                    ctx.indent(),
                    self.name,
                    " : {}".format(", ".join(["public " + x.buildHpp(ctx) for x in self.bases])) if self.bases else "",
                ),
                field_predeclaration,
                "\n".join(body),
                ctx.indent() + "};",
            ])


class Return(CodeStatement):
    _fields = ["value"]

    def __init__(self, value):
        self.value = value

    def buildCpp(self, ctx):
        if self.value:
            return ctx.indent() + "return {};".format(self.value.buildCpp(ctx))
        else:
            return ctx.indent() + "return;"


class Assign(CodeStatement):
    _fields = ["targets", "value"]

    def __init__(self, targets, value):
        self.targets = targets
        self.value = value

    # todo a, b, c = 1, 2, 3
    def buildCpp(self, ctx):
        return ctx.indent() + "{} = {};".format(
            " = ".join([x.buildCpp(ctx) for x in self.targets]),
            self.value.buildCpp(ctx)
        )


class AnnAssign(CodeStatement):
    _fields = ["target", "value", "annotation"]

    def __init__(self, target, value, annotation):
        self.target = target
        self.value = value
        self.annotation = annotation

    def buildCpp(self, ctx):
        if self.value:
            return ctx.indent() + "{} {} = {};".format(
                self.annotation.buildCpp(ctx),
                self.target.buildCpp(ctx),
                self.value.buildCpp(ctx)
            )
        else:
            return ctx.indent() + "{} {};".format(
                self.annotation.buildCpp(ctx),
                self.target.buildCpp(ctx)
            )


class AugAssign(CodeStatement):
    _fields = ["target", "op", "value"]

    def __init__(self, target, op, value):
        self.target = target
        self.op = op
        self.value = value

    def buildCpp(self, ctx):
        return ctx.indent() + "{} {}= {};".format(
            self.target.buildCpp(ctx),
            self.op,
            self.value.buildCpp(ctx)
        )


class For(CodeStatement):
    _fields = ["target", "iter", "body", "orelse"]

    def __init__(self, target, iter, body, orelse):
        super(For, self).__init__(body)
        self.target = target
        self.iter = iter
        self.orelse = orelse
        self.body = self.stmt

    def buildCpp(self, ctx):
        with BuildContext(ctx, self) as new_ctx:
            body = [x.buildCpp(new_ctx) for x in self.stmt]
            # TODO: orelse
            return "\n".join(["{}for (auto {} : {}) {{".format(
                ctx.indent(),
                self.target.buildCpp(ctx),
                self.iter.buildCpp(ctx)
            ),
                "\n".join(body),
                ctx.indent() + "}",
            ])


class While(CodeStatement):
    _fields = ["test", "body", "orelse"]

    def __init__(self, test, body, orelse):
        super(While, self).__init__(body)
        self.test = test
        self.orelse = orelse

    def buildCpp(self, ctx):
        with BuildContext(ctx, self) as new_ctx:
            body = [x.buildCpp(new_ctx) for x in self.stmt]
            # TODO: orelse
            return "\n".join(["{}while ({}) {{".format(
                ctx.indent(),
                self.test.buildCpp(ctx)
            ),
                "\n".join(body),
                ctx.indent() + "}",
            ])


class If(CodeStatement):
    _fields = ["test", "body", "orelse"]

    def __init__(self, test, body, orelse):
        super(If, self).__init__(body)
        self.test = test
        self.orelse = orelse

    def buildCpp(self, ctx):
        with BuildContext(ctx, self) as new_ctx:
            body = [x.buildCpp(new_ctx) for x in self.stmt]
            result = [
                "{}if ({}) {{".format(
                    ctx.indent(),
                    self.test.buildCpp(ctx)
                ),
                "\n".join(body),
                ctx.indent() + "}",
            ]
            if len(self.orelse) == 1 and self.orelse[0].__class__ == If:
                lines = self.orelse[0].buildCpp(ctx).split("\n")
                assert len(lines) > 1
                result[-1] = "}} else {}".format(lines[0])
                result.extend(lines[1:])
            elif self.orelse:
                result[-1] = "} else {"
                result.extend([
                              ] + [x.buildCpp(ctx) for x in self.orelse] + [
                                  "}",
                              ])
            return "\n".join(result)


class Raise(CodeStatement):
    if six.PY3:
        _fields = ["exc", "cause"]
    else:
        _fields = ["type", "inst", "tback"]

    def __init__(self, **kwargs):
        if six.PY3:
            self.exc = kwargs.get("exc")
            self.cause = kwargs.get("cause")
        elif six.PY2:
            self.type = kwargs.get("type")
            self.inst = kwargs.get("inst")
            self.tback = kwargs.get("tback")

    def buildCpp(self, ctx):
        if six.PY3:
            return ctx.indent() + "throw {}();".format(self.exc.buildCpp(ctx))
        elif six.PY2:
            return ctx.indent() + "throw {}();".format(self.type.buildCpp(ctx))


class Expr(CodeStatement):
    _fields = ["value"]

    def __init__(self, value):
        super(Expr, self).__init__(value)
        self.value = value
        del self.stmt

    def buildCpp(self, ctx):
        return ctx.indent() + "{};".format(self.value.buildCpp(ctx))


class Pass(CodeStatement):
    def __init__(self):
        pass

    def buildCpp(self, ctx):
        return ""


class Break(CodeStatement):
    def __init__(self):
        pass

    def buildCpp(self, ctx):
        return ctx.indent() + "break;"


class Continue(CodeStatement):
    def __init__(self):
        pass

    def buildCpp(self, ctx):
        return ctx.indent() + "continue;"


class CodeExpression(Base):
    def __init__(self):
        super(CodeExpression, self).__init__(Type.Expr)


class BoolOp(CodeExpression):
    _fields = ["op", "values"]

    def __init__(self, op, values):
        self.op = op
        self.values = values

    def buildCpp(self, ctx):
        values = []
        for value in self.values:
            if isinstance(value, BoolOp):
                values.append("({})".format(value.buildCpp(ctx)))
            else:
                values.append(value.buildCpp(ctx))
        return " {} ".format(self.op).join(values)


class BinOp(CodeExpression):
    _fields = ["left", "op", "right"]

    def __init__(self, left, op, right):
        super(BinOp, self).__init__()
        self.left = left
        self.op = op
        self.right = right

    def buildCpp(self, ctx):
        return " ".join([self.left.buildCpp(ctx), self.op, self.right.buildCpp(ctx)])


class UnaryOp(CodeExpression):
    _fields = ["op", "operand"]

    def __init__(self, op, operand):
        self.op = op
        self.operand = operand

    def buildCpp(self, ctx):
        operand = self.operand.buildCpp(ctx)
        if isinstance(self.operand, BoolOp):
            operand = "({})".format(operand)
        return "{}{}".format(self.op, operand)


class Lambda(CodeExpression):
    _fields = ["args", "body"]

    def __init__(self, args, body):
        self.args = args
        self.body = body

    def buildCpp(self, ctx):
        args = self.args.buildCpp(ctx)
        body = self.body.buildCpp(ctx)
        return "[&]({}) -> auto {{ return {}; }}".format(args, body)


class IfExp(CodeExpression):
    _fields = ["test", "body", "orelse"]

    def __init__(self, test, body, orelse):
        self.test = test
        self.body = body
        self.orelse = orelse

    def buildCpp(self, ctx):
        test = self.test.buildCpp(ctx)
        body = self.body.buildCpp(ctx)
        orelse = self.orelse.buildCpp(ctx)
        return "(({}) ? ({}) : ({}))".format(test, body, orelse)


class Compare(CodeExpression):
    def __init__(self, left, ops, comparators):
        self.left = left
        self.ops = ops
        self.comparators = comparators

    def buildCpp(self, ctx):
        temp = [self.left.buildCpp(ctx)]
        for op, comp in zip(self.ops, self.comparators):
            temp += [op, comp.buildCpp(ctx)]
        return " ".join(temp)


class Call(CodeExpression):
    _fields = ["func", "args", "keywords", "starargs", "kwargs"]

    def __init__(self, func, args=None, keywords=None, starargs=None, kwargs=None):
        if keywords is None:
            keywords = []
        if args is None:
            args = []
        self.func = func
        self.args = args
        self.keywords = keywords
        self.starargs = starargs
        self.kwargs = kwargs

    def buildCpp(self, ctx):
        # TODO DEVICE_DO
        # if self.func.buildCpp(ctx) == "device_allocator->device_do":
        #     return "device_allocator->template device_do<{}, &{}::{}>({})"
        args = ", ".join([x.buildCpp(ctx) for x in self.args])
        return "{}({})".format(self.func.buildCpp(ctx), args)


class Num(CodeExpression):
    _fields = ["n"]

    def __init__(self, n):
        super(Num, self).__init__()
        self.n = n

    def buildCpp(self, ctx):
        return "{}".format(self.n)


class Str(CodeExpression):
    _fields = ["s"]

    def __init__(self, s):
        self.s = s

    def buildCpp(self, ctx):
        return "\"{}\"".format(self.s.replace('"', '\\"'))


class NameConstant(CodeExpression):
    _fields = ["value"]

    def __init__(self, value):
        self.value = value

    def buildCpp(self, ctx):
        # boolean special case
        if type(self.value) == bool:
            return "true" if self.value else "false"
        return self.value


class Attribute(CodeExpression):
    _fields = ["value", "attr"]

    def __init__(self, value, attr):
        super(Attribute, self).__init__()
        self.value = value
        self.attr = attr

    def buildCpp(self, ctx):
        if self.value.buildCpp(ctx) == "math":
            return self.attr
        if self.value.buildCpp(ctx) == "__pyallocator__":
            return "{}->{}".format("device_allocator", self.attr)
        return "{}->{}".format(self.value.buildCpp(ctx), self.attr)


class Subscript(CodeExpression):
    _fields = ["value", "slice"]

    def __init__(self, value, slice):
        super(Subscript).__init__()
        self.value = value
        self.slice = slice

    def buildCpp(self, ctx):
        return "{}[{}]".format(self.value.buildCpp(ctx), self.slice.buildCpp(ctx))


class Name(CodeExpression):
    _fields = ["id"]

    def __init__(self, id):
        super(Name, self).__init__()
        self.id = id

    def buildCpp(self, ctx):
        # boolean special case
        if self.id == "True":
            return "true"
        elif self.id == "False":
            return "false"
        return self.id

    def buildHpp(self, ctx):
        return self.id


class List(CodeExpression):
    _fields = ["elts"]

    def __init__(self, elts):
        assert False

    def buildCpp(self, ctx):
        assert False


class Tuple(CodeExpression):
    _fields = ["elts"]

    def __init__(self, elts):
        assert False

    def buildCpp(self, ctx):
        assert False


# slice

class Index(CodeExpression):
    _fields = ["value"]

    def __init__(self, value):
        super(Index, self).__init__()
        self.value = value

    def buildCpp(self, ctx):
        return self.value.buildCpp(ctx)


class arguments(Base):
    _fields = ["args", "vararg", "kwarg", "defaults"]

    def __init__(self, args, vararg, kwarg, defaults):
        self.args = args
        self.vararg = vararg
        self.kwarg = kwarg
        self.defaults = defaults
        self.types = {}

    def get_arg_names(self, ctx):
        return [x.buildCpp(ctx) for x in self.args]

    def get_arg_values(self, ctx):
        return [x.buildCpp(ctx) for x in self.defaults]

    def set_arg_type(self, name, type):
        # assert name in self.get_arg_names(ctx)
        self.types[name] = type

    def buildCpp(self, ctx):
        types = dict(self.types)
        names = self.get_arg_names(ctx)
        values = self.get_arg_values(ctx)
        for arg in self.args:
            name = arg.buildCpp(ctx)
            if arg.annotation:
                types[name] = arg.annotation.buildCpp(ctx)
            if name not in types:
                # not defined
                # todo
                types[name] = "int"
        start = len(names) - len(values)
        args = []
        for i, name in enumerate(names):
            tp = types[name]
            tp = CppTypeRegistry.detect(tp)
            if i < start:
                args.append("{} {}".format(tp, name))
            else:
                value = values[i - start]
                args.append("{} {}={}".format(tp, name, value))
        if ctx.is_class_method() and names[0] == "self":
            args = args[1:]
        return ", ".join(args)

    def buildHpp(self, ctx):
        return self.buildCpp(ctx)


class arg(Base):
    _fields = ["arg", "annotation"]

    def __init__(self, arg, annotation=None):
        self.arg = arg
        self.annotation = annotation

    def buildCpp(self, ctx):
        return self.arg


class keyword(Base):
    _fields = ["name", "value"]

    def __init__(self, name, value):
        self.name = name
        self.value = value

    def buildCpp(self, ctx):
        return "static const auto {} = {}".format(
            self.name,
            self.value.buildCpp(ctx)
        )


#
# for C++ syntax special case
#

class CppScope(Attribute):
    def buildCpp(self, ctx):
        return "{}::{}".format(self.value.buildCpp(ctx), self.attr)


class StdCout(Expr):
    def buildCpp(self, ctx):
        temp = ["std::cout"]
        temp += [x.buildCpp(ctx) for x in self.value.args]
        temp += ["std::endl"]
        return ctx.indent() + " << ".join(temp) + ";"


#
# type registry
#
class TypeRegistry(object):
    def __init__(self):
        self.type_map = {}

    def __contains__(self, v):
        return v in self.type_map

    def convert(self, type_str):
        raise NotImplementedError

    def register(self, pytype, cpptype):
        self.type_map[pytype] = cpptype


class CppTypeRegistry(TypeRegistry):
    def convert(self, type):
        return self.type_map[type]

    @staticmethod
    def detect(type, rettype=False):
        if type is None:
            return "void"
        elif type not in type_registry:
            # todo
            return type + "*"
        return type_registry.convert(type)


type_registry = CppTypeRegistry()

# built-in types
type_registry.register("bool", "bool")
type_registry.register("int", "int")
# type_registry.register("long", "long")
type_registry.register("float", "float")
# type_registry.register("complex", "std::complex<double>")
# type_registry.register("str", "std::string")
# type_registry.register("bytearray", "std::string")
# type_registry.register("list", "std::vector")
# type_registry.register("List[int]", "std::vector<int>")
# type_registry.register("tuple", "std::tuple")
