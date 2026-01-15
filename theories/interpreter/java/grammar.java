package com.lox;
package com.tool;

// file to generate Expr.java
import java.io.IOException;
import java.io.PrintWriter;
import java.util.Arrays;
import java.util.List;

// ast abstract syntax tree
abstract class Expr {
    static class Binary extends Expr {
        Binary(Expr left, Token op, Expr right) {
            this.left = left;
            this.op = op;
            this.right = right;
        }
        final Expr left;
        final Token op;
        final Expr right;
    }
}

private static void defineType(
    PrintWriter writer, String baseName, String className, String fieldList) {
        writer.println(" static class " + className + " extends " + baseName + " {");
        writer.println("    " + className + "(" + fieldList + ") {");
        String[] fields = fieldList.split(", ");
        for (String field : fields) {
            String name = field.split(" ")[1];
            writer.println("    this." + name + " = " + name + ";");
        }
        writer.println("    }");
        writer.println();
        writer.println("    @Override");
        // void accept(ExprVisitor visitor), interface Visitor<R>
        writer.println("    <R> R accept(Visitor<R> visitor) {");
        // return visitor.visitGroupingExpr(this);
        writer.println("      return visitor.visit" + className + baseName + "(this);");
        writer.println("    }");
        writer.println();
        for (String field : fields) {
            writer.println("    final " + field + ";");
        }
        writer.println("    }");
    }
// visitor row and col overloading oop
// interface
// abstract class Expr{} 
//      class binaryExpr extends Expr{}, 
//      class groupingExpr extends Expr{}
// interface ExprVisitor{
//      void visitBinary(Binary binary); 
//      void visitGrouping(Grouping grouping)}
// accept(): abstract class Expr{
//              abstract void accept(ExprVisitor vistor)}, 
//      class binaryExpr extends Expr{
//          @Override 
//          void accept(ExprVisitor visitor){
//              visitor.visitBinary(this)}}
private static void defineVisitor(
    PrintWriter writer, String baseName, List<String> types){
        // interface Expr.Visitor
        writer.println("    interface Visitor<R> {");
        for (String type :types) {
            String typeName = type.split(":")[0].trim();
            // void visitGroupingExpr(Expr.Grouping expr);
            writer.println("    R visit" + typeName + baseName + "(" + typeName + baseName.toLowerCase() + ");");
        }
        writer.println("    }");
    }

public static void defineAst (
    String outputDir, String baseName, List<String> types)
    throws IOException {
        String path = outputDir + "/" + baseName + ".java";
        PrintWriter writer = new PrintWriter(path, "UTF-8");
        writer.println("package com.lox;");
        writer.println();
        writer.println("import java.util.List;");
        writer.println();
        writer.println("abstract class " + baseName + " {");

        defineVisitor(writer, baseName, types);
        for (String type : types) {
            String className = type.split(":")[0].trim();
            String fields = type.split(":")[1].trim();
            defineType(writer, baseName, className, fields);
        }
        writer.println();
        // abstract void accept(ExprVisitor visitor); 
        writer.println("    abstract <R> R accept(Visitor<R> visitor);");
        writer.println("}");
        writer.close();
}
public class GenerateAst {
    public static void main(String[] args) throws IOException {
        if (args.length != 1) {
            System.err.println("generate_ast <output dir>");
            System.exit(64);
        }
        String outputDir = arg[0];
        defineAst(outputDir, "Expr", Arrays.asList(
            "Binary : Expr left, Token op, Expr right",
            "Grouping: Expr expression",
            "Literal: Object val",
            "Unary: Token operator, Expr right"
        ));
    }
}

class AstPrinter implements Expr.Visitor<String> {
    String print(Expr expr) {
        // print subexpr
        return expr.accept(this);
    }
    private String parenthesize(String name, Expr ... exprs) {
        StringBuilder builder = new StringBuilder();
        builder.append("(").append(name);
        for (Expr expr : exprs) {
            builder.append(" ");
            // invoke recursively the Visitor pattern to print sub-subexpr til the root
            builder.append(expr.accept(this));
        }
        builder.append(")");
        return builder.toString();
    }
    @Override
    public String visitBinaryExpr(Expr.Binary expr) {
        return parenthesize(expr.operator.lexeme, expr.left, expr.right);
    }
    @Override
    public String visitGroupingExpr(Expr.Grouping expr) {
        return parenthesize("group", expr.expression);
    }
    @Override
    public String visitLiteralExpr(Expr.Literal expr) {
        if (expr.value == null) return "nil";
        return expr.value.toString();
    }
}
// -123 * (45.67)
public static void main(String[] args) {
    Expr expr = new Expr.Binary(
        new Expr.Unary(
            // scanner.java Token(type, lexeme, literal, line)
            new Expr.Token(TokenType.MINUS, '-', null, 1),
            new Expr.Literal(123)),
        new Token(TokenType.STAR, "*", null, 1),
        new Expr.Grouping(
            new Expr.Literal(45.67)));
    System.out.println(new AstPrinter().print(expr));
}
