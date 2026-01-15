package com.lox;

// error handling
public class Lox {
    static boolean hadError = false;
}
static void runFile() {
    run(new String(bytes, Charset.defaultCharset()));
    if (hadError) System.exit(65);
}
static void runPrompt() {
    run(line);
    hadError = false;
}

static void error(int line, String message) {
    report(line, "", message);
}
private static void report(int line, String where, String message) {
    System.err.println(
        "[line " + line + "] Error" + where + ": " + message);
    hadError = true;
}

// token type
enum TokenType {
    // single-character/two char tokens
    LEFT_PAREN, RIGHT_PAREN, LEFT_BRACE, RIGHT_BRACE,COMMA, DOT, MINUS, PLUS, SEMICOLON, SLASH, STAR,
    BANG, BANG_EQUAL, EQUAL, EQUAL_EQUAL, GREATER, GREATER_EQUAL, LESS, LESS_EQUAL,
    // literals
    IDENTIFIER, STRING, NUMBER,
    // keywords
    AND, CLASS, ELSE, FALSE, FUN, FOR, IF, NIL, OR, PRINT, RETURN, SUPER, THIS, TRUE, VAR, WHILE,
    EOF
}
class Token {
    final TokenType type;
    final String lexeme;
    final Object literal;
    final int line;
    Token(TokenType type, String lexeme, Object literal, int line) {
        this.type = type;
        this.lexeme = lexeme;
        this.literal = literal;
        this.line = line;
    }
    public String toString() {
        return type + " " + lexeme + " " + literal;
    }
}

// scanner
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import static com.lox.TokenType.*;

class Scanner {
    private final String source;
    private final List<Token> tokens = new ArrayList<>();
    private int start = 0;
    private int cur = 0;
    private int line = 1;
    private boolean isAtEnd() {
        return cur >= source.length();
    }
    private char advance() {
        return source.charAt(cur++);
    }
    private void addToken(TokenType type) {
        addToken(type, null);
    }
    private void addToken(TokenType type, Object literal) {
        String text = source.substring(start, cur);
        tokens.add(new Token(type, text, literal, line));
    }
    private boolean match(char expected) {
        if (isAtEnd()) return false;
        if (source.charAt(cur) != expected) return false;
        cur++;
        return true;
    }
    private char peek() {
        if (isAtEnd()) return '\0';
        return source.charAt(cur);
    }
    private void string() {
        while (peek()!= '"' && !isAtEnd()) {
            // multi-line str
            if (peek() == '\n') line++;
            advance();
        }
        if (isAtEnd()) {
            Lox.error(line, "unterminated str");
            return;
        }
        advance();
        // substr stripping off the quotes
        String val = source.substring(start+1, cur-1)
        addToken(STRING, val);
    }
    private boolean isDigit(char c) {
        return c >= '0' && c <= '9';
    }
    private peekNext() {
        if (cur+1 >= source.length()) return '\0';
        return source.charAt(cur+1);
    }
    private void number() {
        while (isDigit(peek())) advance();
        // fraction
        if (peek() == '.' && isDigit(peekNext())) {
            // consume . if peekNext is a digit, lookahead
            advance();
            while (isDigit(peek())) advance();
        }
        addToken(NUMBER, Double.parseDouble(source.substring(start, cur)));
    }
    private boolean isAlpha(char c) {
        return (c >= 'a' && c <= 'z') ||
                c == '_';
    }
    private boolean isAlphaNumeric(char c) {
        return isAlpha(c) || isDigit(c);
    }
    private void identifier() {
        while (isAlphaNumeric(peek())) advance();
        String text = source.substring(start, cur);
        TokenType type = keywords.get(text);
        // user-defined type identifier, not in keywords
        if (type == null) type = IDENTIFIER;
        addToken(type);
    }
    private static final Map<String, TokenType> keywords;
    static {
        keywords = new HashMap<>();
        keywords.put("and", AND);
        keywords.put("class", CLASS);
        keywords.put("nil", NIL);
        keywords.put("while", WHILE);
    }
    private void scanToken() {
        char c = advance();
        switch(c) {
            case '(': addToken(LEFT_PAREN); break;
            case ',': addToken(COMMA); break;
            case '!':
                addToken(match('=') ? BANG_EQUAL : BANG);
                break;
            case '/':
                // the next -> comment
                if (match('/')) {
                    // comment->end of line \n, peek until the end, skip, lookahead
                    while (peek() != '\n' && !isAtEnd()) advance();
                } else {
                    addToken(SLASH);
                }
                break;
            case '\t':
                // whitespace, ignore
                break;
            // ...
            case '\n':
                line++;
                break;
            case '"': string(); break;
            default:
                if (isDigit(c)) {
                    number();
                } else if (isAlpha(c)) {
                    identifier();
                } else {
                    Lox.error(line, "unexpected char");
                    break;
                }
        }
    }
    Scanner(String source) {
        this.source = source;
    }
}
List<Token> scanTokens() {
    while (!isAtEnd()) {
        start = cur;
        scanToken();
    }
    tokens.add(new Token(EOF, "", null, line));
    return tokens;
}
