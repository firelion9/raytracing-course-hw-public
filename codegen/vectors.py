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
        + ", ".join(map(lambda f: f"{op}vec.{f}()", fields))
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
        + ", ".join(map(lambda f: f"vec.{f}() {op} scl", fields))
        + "};\n}\n\n"
    )

    res += (
        f"[[nodiscard]] inline {vec_type} operator{op}({scalar_type} scl, const {vec_type}& vec) "
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
        f"[[nodiscard]] inline {vec_type}& operator{op}({vec_type}& vec, {scalar_type} scl) "
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
        f"[[nodiscard]] inline {vec_type} operator{op}(const {vec_type}& a, const {vec_type}& b) "
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
        f"inline {vec_type}& operator{op}({vec_type}& a, const {vec_type}& b) " + "{\n"
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
        f"[[nodiscard]] inline {vec_type} {op}(const {vec_type}& a, const {vec_type}& b) "
        + "{\n"
    )
    res += (
        f"{indent}return " + "{"
        + ", ".join(map(lambda f: f"std::{op}(a.{f}(), b.{f}())", fields))
        + "};\n"
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

    res += (
        f"[[nodiscard]] inline {scalar_type} dot(const {vec_type} &a, const {vec_type} &b) "
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

    return res


def gen_access_ops(fields, type_builder):
    res = ""

    scalar_type = type_builder(1)
    for f in range(len(fields)):
        accessor_name = fields[f]
        res += f"[[nodiscard]] inline {scalar_type}& {accessor_name}() " + "{\n"
        res += f"{indent}return this->val[{f}];\n"
        res += "}\n\n"
        res += (
            f"[[nodiscard]] inline const {scalar_type}& {accessor_name}() const" + "{\n"
        )
        res += f"{indent}return this->val[{f}];\n"
        res += "}\n\n"

    for l in range(2, len(fields) + 1):
        res_type = type_builder(l)
        if res_type == None: continue
        
        for comb in itertools.product(fields, repeat=l):
            accessor_name = "".join(comb)
            res += f"[[nodiscard]] inline {res_type} {accessor_name}() const " + "{\n"
            res += (
                indent
                + "return {"
                + ", ".join(map(lambda f: f"this->{f}()", comb))
                + "};\n"
            )
            res += "}\n\n"

    return res


def gen_vec_member_ops(fields, type_builder):
    res = gen_access_ops(fields, type_builder)
    scalar_type = type_builder(1)
    vec_type = type_builder(len(fields))

    res += f"[[nodiscard]] inline {scalar_type} len2() const " + "{\n"
    res += (
        f"{indent}return "
        + "+".join(map(lambda f: f"this->{f}() * this->{f}()", fields))
        + ";\n"
    )
    res += "}\n\n"

    res += f"[[nodiscard]] inline {scalar_type} len() const " + "{\n"
    res += f"{indent}return std::sqrt(this->len2());\n"
    res += "}\n\n"

    res += f"[[nodiscard]] inline {vec_type} abs() const " + "{\n"
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
        f"{indent} inline {vec_type}("
        + ", ".join(map(lambda f: f"{field_type} {f} = 0", fields))
        + ") : val({"
        + ", ".join(fields)
        + "}) "
        + "{}\n"
    )
    res += f"{indent}inline {vec_type}(const {vec_type}&) = default;\n"
    res += f"{indent}inline {vec_type}& operator=(const {vec_type}&) = default;\n"
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

    res += (
        f"template<class vec_type> [[nodiscard]] inline vec_type norm(const vec_type& vec) "
        + "{\n"
    )
    res += f"{indent}return vec / vec.len();\n"
    res += "}\n\n"

    return res
