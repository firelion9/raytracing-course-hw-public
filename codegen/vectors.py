import textwrap
import itertools

indent = " " * 4


def gen_vector_pure_unary_op(vec_type, fields, op):
    res = (
        f"[[nodiscard]] constexpr inline {vec_type} operator{op}(const {vec_type}& vec) " + "{\n"
    )
    res += (
        indent
        + "return {"
        + ", ".join(map(lambda f: f"{op}vec.{f}()", fields))
        + "};\n}\n"
    )

    return res


def gen_vector_pure_scalar_op(vec_type, fields, scalar_type, op):
    res = (
        f"[[nodiscard]] constexpr inline {vec_type} operator{op}(const {vec_type}& vec, {scalar_type} scl) "
        + "{\n"
    )
    res += (
        indent
        + "return {"
        + ", ".join(map(lambda f: f"vec.{f}() {op} scl", fields))
        + "};\n}\n\n"
    )

    res += (
        f"[[nodiscard]] constexpr inline {vec_type} operator{op}({scalar_type} scl, const {vec_type}& vec) "
        + "{\n"
    )
    res += (
        indent
        + "return {"
        + ", ".join(map(lambda f: f"scl {op} vec.{f}() ", fields))
        + "};\n}\n"
    )

    return res


def gen_vector_modifying_scalar_op(vec_type, fields, scalar_type, op):
    res = (
        f"constexpr inline {vec_type}& operator{op}({vec_type}& vec, {scalar_type} scl) "
        + "{\n"
    )
    res += (
        textwrap.indent(
            "\n".join(map(lambda f: f"vec.{f}() {op} scl;", fields)), indent
        )
        + "\n"
    )
    res += "return vec;\n"
    res += "}\n"

    return res


def gen_vector_pure_op(vec_type, fields, op):
    res = (
        f"[[nodiscard]] constexpr inline {vec_type} operator{op}(const {vec_type}& a, const {vec_type}& b) "
        + "{\n"
    )
    res += (
        indent
        + "return {"
        + ", ".join(map(lambda f: f"a.{f}() {op} b.{f}()", fields))
        + "};\n}\n"
    )

    return res


def gen_vector_modifying_op(vec_type, fields, op):
    res = (
        f"constexpr inline {vec_type}& operator{op}({vec_type}& a, const {vec_type}& b) " + "{\n"
    )
    res += (
        textwrap.indent(
            "\n".join(map(lambda f: f"a.{f}() {op} b.{f}();", fields)), indent
        )
        + "\n"
    )
    res += f"{indent}return a;\n"
    res += "}\n"

    return res


def gen_vector_stream_op(vec_type, fields, op, stream_type):
    res = (
        f"inline {stream_type}& operator{op}({stream_type}& stream, {vec_type}& vec) "
        + "{\n"
    )
    res += (
        f"{indent}return stream {op} "
        + f" {op} ".join(map(lambda f: f"vec.{f}()", fields))
        + ";\n"
    )
    res += "}\n"

    return res

def gen_vector_componentwise_fn(vec_type, fields, op):
    res = (
        f"[[nodiscard]] constexpr inline {vec_type} {op}(const {vec_type}& a, const {vec_type}& b) "
        + "{\n"
    )
    res += (
        f"{indent}return " + "{"
        + ", ".join(map(lambda f: f"std::{op}(a.{f}(), b.{f}())", fields))
        + "};\n"
    )
    res += "}\n"

    return res

def gen_vector_componentwise_scalar_fn(vec_type, scalar_type, fields, op):
    res = (
        f"[[nodiscard]] constexpr inline {vec_type} {op}(const {vec_type}& a, {scalar_type} scl) "
        + "{\n"
    )
    res += (
        f"{indent}return " + "{"
        + ", ".join(map(lambda f: f"std::{op}(a.{f}(), scl)", fields))
        + "};\n"
    )
    res += "}\n"

    return res

def gen_vector_reduce_op(vec_type, fields, return_type, name, zip_op, reduce_op):
    res = f"[[nodiscard]] constexpr inline {return_type} {name}(const {vec_type}& a, const {vec_type}& b) " + "{\n"
    res += (
        f"{indent}return "
        + reduce_op.join(map(lambda f: f"(a.{f}(){zip_op}b.{f}())", fields))
        + ";\n"
    )
    res += "}\n"

    return res

def gen_vector_ops(vec_type, fields, scalar_type):
    res = ""

    res += gen_vector_reduce_op(vec_type, fields, "bool", "operator==", "==", " & ") + "\n"

    ops = ["+", "-", "*", "/"]
    for op in ops:
        res += gen_vector_pure_op(vec_type, fields, op) + "\n"
    for op in ops:
        res += gen_vector_modifying_op(vec_type, fields, op + "=") + "\n"

    res += gen_vector_pure_unary_op(vec_type, fields, "-") + "\n"
    for op in ops:
        res += gen_vector_pure_scalar_op(vec_type, fields, scalar_type, op) + "\n"
    for op in ops:
        res += gen_vector_modifying_scalar_op(vec_type, fields, scalar_type, op + "=") + "\n"

    res += gen_vector_stream_op(vec_type, fields, ">>", "std::istream") + "\n"

    res += (
        f"[[nodiscard]] constexpr inline {scalar_type} dot(const {vec_type} &a, const {vec_type} &b) "
        + "{\n"
    )
    res += (
        f"{indent}return "
        + " + ".join(map(lambda f: f"a.{f}() * b.{f}()", fields))
        + ";\n"
    )
    res += "}\n"

    res += gen_vector_componentwise_fn(vec_type, fields, "min") + "\n"
    res += gen_vector_componentwise_fn(vec_type, fields, "max") + "\n"
    res += gen_vector_componentwise_fn(vec_type, fields, "pow") + "\n"
    res += gen_vector_componentwise_scalar_fn(vec_type, scalar_type, fields, "pow") + "\n"

    return res


def gen_access_ops(fields, type_builder):
    res = ""

    scalar_type = type_builder(1)
    for f in range(len(fields)):
        accessor_name = fields[f]
        res += f"[[nodiscard]] constexpr inline {scalar_type}& {accessor_name}() " + "{\n"
        res += f"{indent}return this->val[{f}];\n"
        res += "}\n\n"
        res += (
            f"[[nodiscard]] constexpr inline const {scalar_type}& {accessor_name}() const" + "{\n"
        )
        res += f"{indent}return this->val[{f}];\n"
        res += "}\n\n"

    for l in range(2, len(fields) + 1):
        res_type = type_builder(l)
        if res_type == None: continue
        
        for comb in itertools.product(fields, repeat=l):
            accessor_name = "".join(comb)
            res += f"[[nodiscard]] constexpr inline {res_type} {accessor_name}() const " + "{\n"
            res += (
                indent
                + "return {"
                + ", ".join(map(lambda f: f"this->{f}()", comb))
                + "};\n"
            )
            res += "}\n\n"

    return res

def gen_partitions(n):
    if n == 0:
        yield []
    else:
        for i in range(1, n + 1):
            for part in gen_partitions(n - i):
                part.append(i)
                yield part

def gen_conv_constructors(fields, type_builder):
    res = ""

    vec_type = type_builder(len(fields))
    for part in gen_partitions(len(fields)):
        types = [type_builder(i) for i in part]
        if len(part) == len(fields) or len(part) == 1 or any(map(lambda t: t is None, types)):
                continue
        res += f"[[nodiscard]] constexpr inline {vec_type}("
        res += ", ".join(map(lambda p: f"{p[1]} p{p[0]}", enumerate(types)))
        res += f") : {vec_type}" + "{"
        res += ", ".join(map(lambda p: ", ".join(map(lambda i: f"p{p[0]}.val[{i}]" if p[1] > 1 else f"p{p[0]}", range(p[1]))), enumerate(part)))
        res += "} {}\n\n"

    return res


def gen_vec_member_ops(fields, type_builder):
    res = gen_access_ops(fields, type_builder)
    scalar_type = type_builder(1)
    vec_type = type_builder(len(fields))

    res += f"[[nodiscard]] constexpr inline {scalar_type} len_l1() const " + "{\n"
    res += (
        f"{indent}return "
        + "+".join(map(lambda f: f"std::abs(this->{f}())", fields))
        + ";\n"
    )
    res += "}\n\n"

    res += f"[[nodiscard]] constexpr inline {scalar_type} len2() const " + "{\n"
    res += (
        f"{indent}return "
        + "+".join(map(lambda f: f"this->{f}() * this->{f}()", fields))
        + ";\n"
    )
    res += "}\n\n"

    res += f"[[nodiscard]] constexpr inline {scalar_type} len() const " + "{\n"
    res += f"{indent}return std::sqrt(this->len2());\n"
    res += "}\n\n"

    res += f"[[nodiscard]] constexpr inline {vec_type} abs() const " + "{\n"
    res += (
        f"{indent}return " + "{"
        + ", ".join(map(lambda f: f"std::abs(this->{f}())", fields))
        + "};\n"
    )
    res += "}\n\n"

    return res


def _gen_vector_def(fields, vec_type, field_type, type_builder):

    res = f"struct {vec_type} " + "{\n"
    res += f"{indent}std::array<{field_type}, {len(fields)}> val;\n\n"
    res += (
        f"{indent}[[nodiscard]] constexpr inline {vec_type}("
        + ", ".join(map(lambda f: f"{field_type} {f} = 0", fields))
        + ") : val({"
        + ", ".join(fields)
        + "}) "
        + "{}\n"
    )
    res += f"{indent}[[nodiscard]] constexpr inline {vec_type}(const {vec_type}&) = default;\n"
    res += f"{indent}inline {vec_type}& operator=(const {vec_type}&) = default;\n"
    res += textwrap.indent(gen_conv_constructors(fields, type_builder), indent)
    res += "\n" + textwrap.indent(
        gen_vec_member_ops(fields, type_builder),
        indent,
    )
    res += "};\n\n"

    res += gen_vector_ops(vec_type, fields, field_type)

    return res


def gen_vector_def(field_count, field_type):
    fields = ["x", "y", "z", "w"][:field_count]
    vec_type = f"vec{field_count}"

    return _gen_vector_def(fields, vec_type, field_type, lambda n: field_type if n == 1 else f"vec{n}")


def gen_color_def(field_count, field_type):
    fields = ["r", "g", "b", "a"][:field_count]
    vec_type = f"color{field_count}"

    return _gen_vector_def(fields, vec_type, field_type, lambda n: field_type if n == 1 else f"color{n}" if n > 2 else None)


def generate():
    res = ""
    res += gen_vector_def(2, "float") + "\n"
    res += gen_vector_def(3, "float") + "\n"
    res += gen_vector_def(4, "float") + "\n"
    res += gen_color_def(3, "float") + "\n"
    res += gen_color_def(4, "float") + "\n"

    return res
