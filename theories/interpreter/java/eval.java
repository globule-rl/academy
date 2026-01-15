// obj -> any types runtime instanceof
// scanning: val -> token, parser: token -> tree node
// eval: literal tree node -> runtime val, pull it out
package com.lox;

class Interpreter implements Expr.Visitor<Object> {
    private Object eval(Expr expr) {
        // visit method: the gut, real work, interface ExprVisitor {}
        // accept(interface): 
        //      abstract class Expr{
        //          abstract void accept(ExprVisitor vistor)},
        // print subexpr, visitor impl
        return expr.accept(this);
    }
    private boolean isTruthy(Object obj) {
        if (obj == null) return false;
        if (obj instanceof Boolean) return (boolean)obj;
        return true;
    }
    private String stringify(Object obj) {
        if (obj == null) return "nil";
        if (obj instanceof Double) {
            String text = obj.toString();
            // int, use double precision
            if (text.endsWith(".0")) {
                text = text.substring(0, text.length() - 2);
            }
            return text;
        }
        return obj.toString();
    }

    //runtimeError.java
    class RuntimeError extends RuntimeException {
        final Token token;
        RuntimeError(Token token, String msg) {
            super(msg);
            this.token = token;
        }
    }
    
    private void checkNumberOperand(Token op, Object operand) {
        if (operand instanceof Double) return;
        throw new RuntimeError(op, "op must be a num");
    }
    private void checkNumberOperands(Token op, Object left, Object right) {
        if (left instanceof Double && right instanceof Double) return;
        throw new RuntimeError(op, "op must be nums");
    }

    @Override
    public Object visitLiteralExpr(Expr.Literal expr) {
        return expr.val;
    }
    @Override
    public Object visitGroupingExpr(Expr.Grouping expr) {
        return eval(expr.expression);
    }
    @Override
    public Object visitUnaryExpr(Expr.Unary expr) {
        // post-order traversal, eval op right/subexpr/children first
        Object right = eval(expr.right);
        switch (expr.op.type) {
            // logical NOT
            case BANG:
                return !isTruthy(right);
            case MINUS:
                checkNumberOperand(expr.op, right);
                // cast it to num at runtime
                return -(double)right;
        }
        return null;
    }
    @Override
    public Object visitBinaryExpr(Expr.Binary expr) {
        Object left = eval(expr.left);
        Object right = eval(expr.right);

        switch (expr.op.type) {
            case MINUS:
                checkNumberOperands(expr.op, left, right);
                return (double)left - (double)right;
            // ...
            case SLASH:
                return (double)left / (double)right;
        }
        return null;
    }
    void interpret(Expr expr) {
        try {
            // visit syntax tree -> ret val/obj
            Object val = eval(expr);
            System.out.println(stringify(val));
        } catch (RuntimeError err) {
            Lox.runtimeError(err);
        }
    }
    // lox.java
    static void runtimeError(RuntimeError error) {
        System.err.prinln(error.getMsg() +
            "\n[line " + error.token.line + "]");
        hadRuntimeError = true;
    }
    public class Lox {
        private static final Interpreter interpreter = new Interpreter();
        static boolean hadRuntimeError = false;
    }
    static void runFile() {
        if (hadRuntimeError) System.exit(70);
    }
    static void run() {
        List<Token> tokens = scanner.scanTokens();
        Parser parser = new Parser(tokens);
        Expr expr = parser.parse();
        
        if (hadError) return;
        interpreter.interpret(expr);
    }

}