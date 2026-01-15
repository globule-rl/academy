// prededence syntax tree
// expression -> equality -> comp((|)comp)* -> term((|)term)* 
// -> factor((|)factor)* -> unary (|)unary|primary -> primary|expression
// (...)* -> while loop

// recursive descent top=down
package com.lox;
import java.util.List;
import static com.lox.TokenType.*;

class Parser {
    private static class ParseError extends RuntimeException {}

    // list: flat inp sequence
    private final List<Token> tokens;
    private int cur = 0;
    Parser(List<Token> tokens) {
        this.tokens = tokens;
    }
    private Token previous() {
        return tokens.get(cur - 1);
    }
    private Token peek() {
        // check, not yet consumed
        return tokens.get(cur);
    }
    private boolean isAtEnd() {
        return peek().type == EOF;
    }
    private Token advance() {
        // consume it
        if (!isAtEnd()) cur++;
        return previous();
    }
    private boolean check(TokenType type) {
        if (isAtEnd()) return false;
        return peek().type == type;
    } 
    private boolean match(TokenType... types) {
        for (TokenType type : types) {
            if (check(type)) {
                advance();
                return true;
            }
        }
        return false;
    }

    // lox.java
    static void error(Token, token, String msg) {
        if (token.type == TokenType.EOF) {
            report(token.line, " at end", msg);
        } else {
            report(token.line, " at '" + token.lexeme + "'", msg);
        }
    }

    private ParseError error(Token token, String msg) {
        Lox.error(token, msg);
        // ret not throw err, let parser decide to unwind or no/synchronize/cascaded err
        return new ParseError();
    }
    private void synchronize() {
        advance();
        while (!isAtEnd()) {
            // dicard token at the ; boundary
            if (previous().type == SEMICOLON) return;
            switch (peek().type) {
                case CLASS:
                case IF:
                case RETURN:
                    return;
            }
            advance();
        }
    }
    private Token consume(TokenType type, String msg) {
        if (check(type)) return advance();
        throw error(peek(), msg);
    }

    private Expr primary() {
        if (match(FALSE)) return new Expr.Literal(false);
        if (match(TRUE)) return new Expr.Literal(true);
        if (match(NIL)) return new Expr.Literal(null);
        if (match(NUMBER, STRING)) {
            return new Expr.Literal(previous().literal);
        }
        if (match(LEFT_PAREN)) {
            Expr expr = expression();
            consume(RIGHT_PAREN, "Expect ')' after expr");
            return new Expr.Grouping(expr);
        }
        throw error(peek(), "Expect expr");
    }

    private Expr unary() {
        if (match(BANG, MINUS)) {
            Token op = previous();
            Expr right = unary();
            return new Expr.Unary(op.right);
        }
        return primary();
    }
    // ...
    private Expr equality() {
        // left-hand operand
        Expr expr = comp();
        while (match(BANG_EQUAL, EQUAL_EQUAL)) {
            Token op = previous();
            // right-hand operand
            Expr right = comp();
            // left-associative nested binary op nodes
            expr = new Expr.Binary(expr, op. right);
        }
        return expr;
    }
    private Expr expression() {
        return equlity();
    }

    Expr parse() {
        try {
            return expression();
        } catch (ParseError error) {
            return null;
        }
    }
}

// lox.java
static void run() {
    List<Token> tokens = scanner.scanTokens();
    Parser parser = new Parser(tokens);
    Expr expr = parser.parse();
    
    if (hadError) return;
    System.out.println(new AstPrinter().print(expr));
}
