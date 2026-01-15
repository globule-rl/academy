#ifdef clox_scanner_h
#define clox_scanner_h

typedef enum {
    TOKEN_FUNC, TOKEN_COMMA,
    TOKEN_LEFT_PAREN, TOKEN_RIGHT_PAREN, 
    TOKEN_LEFT_BRACE, TOKEN_RIGHT_BRACE, 
    TOKEN_BANG, TOKEN_BANG_EQUAL, TOKEN_EQUAL, TOKEN_MINUS, TOKEN_FALSE,
    TOKEN_NUM,TOKEN_STAR,TOKEN_STR, TOKEN_AND, TOKEN_OR, 
    TOKEN_ID, TOKEN_SEMICOLON, TOKEN_FOR,
    TOKEN_IF, TOKEN_ELSE, TOKEN_VAR, TOKEN_WHILE,
    TOKEN_ERR, TOKEN_EOF
} TokenType;

typedef struct {
    TokenType type;
    const char* start;
    const char* cur;
    int length;
    int line;
} Token;

void initScanner(const char* src);
Token scanToken();

#endif