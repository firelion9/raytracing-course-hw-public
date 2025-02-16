import textwrap
import itertools

indent = " " * 4


def gen_vector_pure_unary_op(vec_type, fields, op):
    res = (
        f"[[nodiscard]] inline {vec_type} operator{op}(const {vec_type}& vec) " + "{\n"
    )
    res += (
        indent
        + "return {"
        + ", ".join(map(lambda f: f"{op}vec.{f}", fields))
        + "};\n}\n"
    )

    return res


def gen_vector_pure_scalar_op(vec_type, fields, scalar_type, op):
    res = (
        f"[[nodiscard]] inline {vec_type} operator{op}(const {vec_type}& vec, {scalar_type} scl) "
        + "{\n"
    )
    res += (
        indent
        + "return {"
        + ", ".join(map(lambda f: f"vec.{f} {op} scl", fields))
        + "};\n}\n\n"
    )

    res += (
        f"[[nodiscard]] inline {vec_type} operator{op}({scalar_type} scl, const {vec_type}& vec) "
        + "{\n"
    )
    res += (
        indent
        + "return {"
        + ", ".join(map(lambda f: f"scl {op} vec.{f} ", fields))
        + "};\n}\n"
    )

    return res


def gen_vector_modifying_scalar_op(vec_type, fields, scalar_type, op):
    res = (
        f"[[nodiscard]] inline {vec_type}& operator{op}({vec_type}& vec, {scalar_type} scl) "
        + "{\n"
    )
    res += (
        textwrap.indent("\n".join(map(lambda f: f"vec.{f} {op} scl;", fields)), indent)
        + "\n"
    )
    res += "return vec;\n"
    res += "}\n"

    return res


def gen_vector_pure_op(vec_type, fields, op):
    res = (
        f"[[nodiscard]] inline {vec_type} operator{op}(const {vec_type}& a, const {vec_type}& b) "
        + "{\n"
    )
    res += (
        indent
        + "return {"
        + ", ".join(map(lambda f: f"a.{f} {op} b.{f}", fields))
        + "};\n}\n"
    )

    return res


def gen_vector_modifying_op(vec_type, fields, op):
    res = (
        f"inline {vec_type}& operator{op}({vec_type}& a, const {vec_type}& b) " + "{\n"
    )
    res += (
        textwrap.indent("\n".join(map(lambda f: f"a.{f} {op} b.{f};", fields)), indent)
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
        + f" {op} ".join(map(lambda f: f"vec.{f}", fields))
        + ";\n"
    )
    res += "}\n"

    return res


def gen_vector_ops(vec_type, fields, scalar_type):
    res = ""

    ops = ["+", "-", "*", "/"]
    for op in ops:
        res += gen_vector_pure_op(vec_type, fields, op) + "\n"
    for op in ops:
        res += gen_vector_modifying_op(vec_type, fields, op + "=") + "\n"

    res += gen_vector_pure_unary_op(vec_type, fields, "-") + "\n"
    res += gen_vector_pure_scalar_op(vec_type, fields, scalar_type, "*") + "\n"
    res += gen_vector_pure_scalar_op(vec_type, fields, scalar_type, "/") + "\n"
    res += gen_vector_modifying_scalar_op(vec_type, fields, scalar_type, "*=") + "\n"
    res += gen_vector_modifying_scalar_op(vec_type, fields, scalar_type, "/=") + "\n"

    res += gen_vector_stream_op(vec_type, fields, ">>", "std::istream") + "\n"

    return res


def gen_access_ops(fields, type_builder):
    res = ""
    for l in range(2, len(fields) + 1):
        res_type = type_builder(l)
        for perm in itertools.permutations(fields, l):
            accessor_name = "".join(perm)
            res += f"[[nodiscard]] inline {res_type} {accessor_name}() const " + "{\n"
            res += (
                indent
                + "return {"
                + ", ".join(map(lambda f: f"this->{f}", perm))
                + "};\n"
            )
            res += "}\n\n"

    return res


def gen_vec_member_ops(fields, type_builder):
    res = gen_access_ops(fields, type_builder)
    scalar_type = type_builder(1)

    res += f"[[nodiscard]] inline {scalar_type} len2() const " + "{\n"
    res += (
        f"{indent}return "
        + "+".join(map(lambda f: f"this->{f} * this->{f}", fields))
        + ";\n"
    )
    res += "}\n"

    res += f"[[nodiscard]] inline {scalar_type} len() const " + "{\n"
    res += f"{indent}return sqrt(this->len2());\n"
    res += "}\n"

    return res


def _gen_vector_def(fields, vec_type, field_type):

    res = f"struct {vec_type} " + "{\n"
    res += "\n".join(map(lambda f: f"{indent}{field_type} {f} = 0;", fields))
    res += "\n\n" + textwrap.indent(
        gen_vec_member_ops(fields, lambda n: field_type if n == 1 else f"vec{n}"),
        indent,
    )
    res += "};\n\n"

    res += gen_vector_ops(vec_type, fields, field_type)

    return res


def gen_vector_def(field_count, field_type):
    fields = ["x", "y", "z", "w"][:field_count]
    vec_type = f"vec{field_count}"

    return _gen_vector_def(fields, vec_type, field_type)


def gen_color_def(field_count, field_type):
    fields = ["r", "g", "b", "a"][:field_count]
    vec_type = f"color{field_count}"

    return _gen_vector_def(fields, vec_type, field_type)


def generate():
    res = ""
    res += gen_vector_def(2, "float") + "\n"
    res += gen_vector_def(3, "float") + "\n"
    res += gen_vector_def(4, "float") + "\n"
    res += gen_color_def(3, "float") + "\n"
    res += gen_color_def(4, "float") + "\n"

    return res
