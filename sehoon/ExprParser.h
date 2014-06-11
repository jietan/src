#ifndef SEHOON_EXPR_PARSER_H
#define SEHOON_EXPR_PARSER_H


#include <cstdio>
#include <cstdarg>
#include <cstring>
#include <cctype>
#include <cmath>

#include <vector>

namespace sehoon {
    namespace expr {
        static const int NAME_LEN_MAX = 30;
        static const int EXPR_LEN_MAX = 255;
        static const int ERR_LEN_MAX = 255;

        class Error;
        static double factorial(double value);
        static double sign(double value);
        static void toupper(char upper[], const char str[]);
        class Variablelist;
        class Parser;
        
        class Error {
        public:
            Error(const int row, const int col, const int id, ...)
                : err_row(row), err_col(col), err_id(id) { 
                //sprintf(msg, msgdesc(id));
                const char* const msg_desc = msgdesc(id);

                va_list args;
                va_start(args, id);
                vsnprintf(msg, sizeof(msg)-1, msg_desc, args);
                msg[sizeof(msg)-1] = '\0';
                va_end(args);

                // init(row, col, msg_desc, ...);
                
            }

            int get_row() {return err_row;} // Returns the row of the error
            int get_col() {return err_col;} // Returns the column of the error
            int get_id() {return err_id;}   // Returns the id of the error
            char* get_msg() {return msg;}   // Returns a pointer to the error msg

        private:
            int err_row;    // row where the error occured
            int err_col;    // column (position) where the error occured
            int err_id;     // id of the error
            char msg[255];
        
            const char* const msgdesc(const int id) {
                switch (id)
                {
                    // syntax errors
                case 1: return "Syntax error in part \"%s\"";
                case 2: return "Syntax error";
                case 3: return "Parentesis ) missing";
                case 4: return "Empty expression";
                case 5: return "Unexpected part \"%s\"";
                case 6: return "Unexpected end of expression";
                case 7: return "Value expected";

                    // wrong or unknown operators, functions, variables
                case 101: return "Unknown operator %s";
                case 102: return "Unknown function %s";
                case 103: return "Unknown variable %s";

                    // domain errors
                case 200: return "Too long expression, maximum number of characters exceeded";
        
                    // error in assignments of variables
                case 300: return "Defining variable failed";
        
                    // error in functions
                case 400: return "Integer value expected in function %s";
                }

                return "Unknown error";
            }
        };        

        /*
         * calculate factorial of value
         * for example 5! = 5*4*3*2*1 = 120
         */
        static double factorial(double value) {
            double res;
            int v = static_cast<int>(value);
    
            if (value != static_cast<double>(v))
            {
                throw Error(-1, -1, 400, "factorial");
            }
    
            res = v;
            v--;
            while (v > 1)
            {
                res *= v;
                v--;
            }

            if (res == 0) res = 1;        // 0! is per definition 1
            return res;
        }

        /* 
         * calculate the sign of the given value
         */
        static double sign(double value) {
            if (value > 0) return 1;
            if (value < 0) return -1;
            return 0;
        }

        static void toupper(char upper[], const char str[]) {
            int i = -1;
            do {
                i++;
                upper[i] = std::toupper(str[i]);
            }
            while (str[i] != '\0');

        }
        class Variablelist {
        public:
            bool exist(const char* name) {
                return (get_id(name) != -1);                
            }
            
            bool add(const char* name, double value) {
                VAR new_var;
                strncpy(new_var.name, name, 30);
                new_var.value = value;

                int id = get_id(name);
                if (id == -1)
                {
                    // variable does not yet exist
                    var.push_back(new_var);
                }
                else
                {
                    // variable already exists. overwrite it
                    var[id] = new_var;
                }
                return true;
            }
            
            bool del(const char* name) {
                int id = get_id(name);
                if (id != -1)
                {
                    var[id] = var[var.size()-1]; // move last item to deleted item
                    var.pop_back();              // remove last item
                    return true;
                }
                return false;                 
            }

            bool get_value(const char* name, double* value) {
                int id = get_id(name);
                if (id != -1)
                {
                    *value = var[id].value;
                    return true;
                }
                return false;                
            }
            
            bool get_value(const int id, double* value) {
                if (id >=0 && id < var.size())
                {
                    *value = var[id].value;
                    return true;
                }
                return false;                
            }


            int  get_id(const char* name) {
                // first make the name uppercase
                char nameU[NAME_LEN_MAX+1];
                char varU[NAME_LEN_MAX+1];
                toupper(nameU, name);
    
                for (int i = 0; i < var.size(); i++)
                {
                    toupper(varU, var[i].name);
                    if (strcmp(nameU, varU) == 0)
                    {
                        return i;
                    }
                }
                return -1;                
            }
            
            bool set_value(const char* name, const double value) {
                return add(name, value);
            }

        private:
            struct VAR {
                char name[NAME_LEN_MAX+1];
                double value;
            };

            std::vector<VAR> var;
        };


/*
 * checks if the given char c is a minus
 */
        static bool isMinus(const char c) {
            if (c == 0) return 0;
            return c == '-';
        }



/*
 * checks if the given char c is whitespace
 * whitespace when space chr(32) or tab chr(9)
 */
        static bool isWhiteSpace(const char c) {
            if (c == 0) return 0;
            return c == 32 || c == 9;  // space or tab
        }

/*
 * checks if the given char c is a delimeter
 * minus is checked apart, can be unary minus
 */
        static bool isDelimeter(const char c) {
            if (c == 0) return 0;
            return strchr("&|<>=+/*%^!", c) != 0;
        }

/*
 * checks if the given char c is NO delimeter
 */
        static bool isNotDelimeter(const char c) {
            if (c == 0) return 0;
            return strchr("&|<>=+-/*%^!()", c) != 0;
        }

/*
 * checks if the given char c is a letter or undersquare
 */
        static bool isAlpha(const char c) {
            if (c == 0) return 0;
            return strchr("ABCDEFGHIJKLMNOPQRSTUVWXYZ_", std::toupper(c)) != 0;
        }

/*
 * checks if the given char c is a digit or dot
 */
        static bool isDigitDot(const char c) {
            if (c == 0) return 0;
            return strchr("0123456789.", c) != 0;
        }

/*
 * checks if the given char c is a digit
 */
        static bool isDigit(const char c) {
            if (c == 0) return 0;
            return strchr("0123456789", c) != 0;
        }

        class Parser {
            // public functions
        public:
            Parser() {
                expr[0] = '\0';
                e = NULL;

                token[0] = '\0';
                token_type = NOTHING;
            }

/**
 * parses and evaluates the given expression
 * On error, an error of type Error is thrown
 */
            char* parse(const char new_expr[]) {
                try
                {
                    // check the length of expr
                    if (strlen(new_expr) > EXPR_LEN_MAX)
                    {
                        throw Error(row(), col(), 200);
                    }

                    // initialize all variables
                    strcpy(expr, new_expr);     // copy the given expression to expr
                    e = expr;                   // let e point to the start of the expression
                    ans = 0;

                    getToken();
                    if (token_type == DELIMETER && *token == '\0')
                    {
                        throw Error(row(), col(), 4);
                    }

                    ans = parse_level1();

                    // check for garbage at the end of the expression 
                    // an expression ends with a character '\0' and token_type = delimeter
                    if (token_type != DELIMETER || *token != '\0')
                    {
                        if (token_type == DELIMETER)
                        {
                            // user entered a not existing operator like "//"
                            throw Error(row(), col(), 101, token);
                        }
                        else
                        {
                            throw Error(row(), col(), 5, token);
                        }
                    }  

                    // add the answer to memory as variable "Ans"
                    user_var.add("Ans", ans);

                    snprintf(ans_str, sizeof(ans_str), "Ans = %g", ans);
                }
                catch (Error err)
                {
                    if (err.get_row() == -1)
                    {
                        snprintf(ans_str, sizeof(ans_str), "Error: %s (col %i)", err.get_msg(), err.get_col());
                    }
                    else
                    {
                        snprintf(ans_str, sizeof(ans_str), "Error: %s (ln %i, col %i)", err.get_msg(), err.get_row(), err.get_col());
                    }
                }

                return ans_str;
            }

                
            double getAnswer() const { return ans; }
            Variablelist* getVarList() { return &user_var; }
            // enumerations
        private:
    
            enum TOKENTYPE {NOTHING = -1, DELIMETER, NUMBER, VARIABLE, FUNCTION, UNKNOWN};
    
            enum OPERATOR_ID {AND, OR, BITSHIFTLEFT, BITSHIFTRIGHT,                 // level 2
                              EQUAL, UNEQUAL, SMALLER, LARGER, SMALLEREQ, LARGEREQ,    // level 3
                              PLUS, MINUS,                     // level 4
                              MULTIPLY, DIVIDE, MODULUS, XOR,  // level 5
                              POW,                             // level 6
                              FACTORIAL};                      // level 7

            // data
        private:
            char expr[EXPR_LEN_MAX];    // holds the expression
            char* e;                      // points to a character in expr
        
            char token[NAME_LEN_MAX];   // holds the token
            TOKENTYPE token_type;         // type of the token

            double ans;                   // holds the result of the expression
            char ans_str[255];            // holds a string containing the result 
            // of the expression

            Variablelist user_var;        // list with variables defined by user

            // private functions
        private:
/**
 * Get next token in the current string expr.
 * Uses the Parser data expr, e, token, t, token_type and err
 */
            void getToken() {
                token_type = NOTHING;
                char* t;           // points to a character in token
                t = token;         // let t point to the first character in token
                *t = '\0';         // set token empty

                //printf("\tgetToken e:{%c}, ascii=%i, col=%i\n", *e, *e, e-expr);

                // skip over whitespaces
                while (*e == ' ' || *e == '\t')     // space or tab
                {
                    e++;
                }

                // check for end of expression
                if (*e == '\0')
                {
                    // token is still empty
                    token_type = DELIMETER;
                    return;
                }

                // check for minus
                if (*e == '-')
                {
                    token_type = DELIMETER;
                    *t = *e;
                    e++;
                    t++;
                    *t = '\0';  // add a null character at the end of token
                    return;
                }

                // check for parentheses
                if (*e == '(' || *e == ')')
                {
                    token_type = DELIMETER;
                    *t = *e;
                    e++;
                    t++;
                    *t = '\0';
                    return;
                }

                // check for operators (delimeters)
                if (isDelimeter(*e))
                {
                    token_type = DELIMETER;
                    while (isDelimeter(*e))
                    {
                        *t = *e;
                        e++;
                        t++;
                    }
                    *t = '\0';  // add a null character at the end of token
                    return;
                }
    
                // check for a value
                if (isDigitDot(*e))
                {
                    token_type = NUMBER;
                    while (isDigitDot(*e))
                    {
                        *t = *e;
                        e++;
                        t++;
                    }
        
                    // check for scientific notation like "2.3e-4" or "1.23e50"
                    if (std::toupper(*e) == 'E')
                    {
                        *t = *e;
                        e++;
                        t++;
    
                        if (*e == '+' || *e == '-')
                        {
                            *t = *e;
                            e++;
                            t++;
                        }

                        while (isDigit(*e))
                        {
                            *t = *e;
                            e++;
                            t++;
                        }
                    }
        
                    *t = '\0';
                    return;
                }

                // check for variables or functions
                if (isAlpha(*e))
                {
                    while (isAlpha(*e) || isDigit(*e))
                        //while (isNotDelimeter(*e))
                    {
                        *t = *e;
                        e++;
                        t++;
                    }
                    *t = '\0';  // add a null character at the end of token

                    // check if this is a variable or a function.
                    // a function has a parentesis '(' open after the name 
                    char* e2 = NULL;
                    e2 = e;

                    // skip whitespaces
                    while (*e2 == ' ' || *e2 == '\t')     // space or tab
                    {
                        e2++;
                    }
        
                    if (*e2 == '(') 
                    {
                        token_type = FUNCTION;
                    }
                    else
                    {
                        token_type = VARIABLE;
                    }
                    return;
                }

                // something unknown is found, wrong characters -> a syntax error
                token_type = UNKNOWN;
                while (*e != '\0')
                {
                    *t = *e;
                    e++;
                    t++;
                }
                *t = '\0';
                throw Error(row(), col(), 1, token);

                return;
            }


/*
 * assignment of variable or function
 */
            double parse_level1() {
                if (token_type == VARIABLE)
                {
                    // copy current token
                    char* e_now = e;
                    TOKENTYPE token_type_now = token_type;
                    char token_now[NAME_LEN_MAX+1];
                    strcpy(token_now, token);
        
                    getToken();
                    if (strcmp(token, "=") == 0)
                    {
                        // assignment
                        double ans;
                        getToken();
                        ans = parse_level2();
                        if (user_var.add(token_now, ans) == false)
                        {
                            throw Error(row(), col(), 300);
                        }
                        return ans;
                    }
                    else
                    {
                        // go back to previous token
                        e = e_now;
                        token_type = token_type_now;
                        strcpy(token, token_now);
                    }
                }

                return parse_level2();
            }


/*
 * conditional operators and bitshift
 */
            double parse_level2() {
                int op_id;
                double ans;
                ans = parse_level3();

                op_id = get_operator_id(token);
                while (op_id == AND || op_id == OR || op_id == BITSHIFTLEFT || op_id == BITSHIFTRIGHT)
                {
                    getToken();
                    ans = eval_operator(op_id, ans, parse_level3());
                    op_id = get_operator_id(token);
                }

                return ans;
            }

/*
 * conditional operators
 */
            double parse_level3() {
                int op_id;
                double ans;
                ans = parse_level4();

                op_id = get_operator_id(token);
                while (op_id == EQUAL || op_id == UNEQUAL || op_id == SMALLER || op_id == LARGER || op_id == SMALLEREQ || op_id == LARGEREQ)
                {
                    getToken();
                    ans = eval_operator(op_id, ans, parse_level4());
                    op_id = get_operator_id(token);
                }

                return ans;
            }

/*
 * add or subtract
 */
            double parse_level4() {
                int op_id;
                double ans;
                ans = parse_level5();
    
                op_id = get_operator_id(token);
                while (op_id == PLUS || op_id == MINUS)
                {
                    getToken();
                    ans = eval_operator(op_id, ans, parse_level5());
                    op_id = get_operator_id(token);
                }

                return ans;
            }


/*
 * multiply, divide, modulus, xor
 */
            double parse_level5() {
                int op_id;
                double ans;
                ans = parse_level6();

                op_id = get_operator_id(token);
                while (op_id == MULTIPLY || op_id == DIVIDE || op_id == MODULUS || op_id == XOR)
                {
                    getToken();
                    ans = eval_operator(op_id, ans, parse_level6());
                    op_id = get_operator_id(token);
                }

                return ans;
            }


/*
 * power
 */
            double parse_level6() {
                int op_id;
                double ans;
                ans = parse_level7();

                op_id = get_operator_id(token);
                while (op_id == POW)
                {
                    getToken();
                    ans = eval_operator(op_id, ans, parse_level7());
                    op_id = get_operator_id(token);
                }

                return ans;
            }

/*
 * Factorial
 */
            double parse_level7() {
                int op_id;
                double ans;
                ans = parse_level8();

                op_id = get_operator_id(token);
                while (op_id == FACTORIAL)
                {
                    getToken();
                    // factorial does not need a value right from the 
                    // operator, so zero is filled in.
                    ans = eval_operator(op_id, ans, 0.0);
                    op_id = get_operator_id(token);
                }

                return ans;
            }

/*
 * Unary minus
 */
            double parse_level8() {
                double ans;
    
                int op_id = get_operator_id(token);    
                if (op_id == MINUS)
                {
                    getToken();
                    ans = parse_level9();
                    ans = -ans;
                }
                else
                {
                    ans = parse_level9();
                }
    
                return ans;
            }


/*
 * functions
 */
            double parse_level9() {
                char fn_name[NAME_LEN_MAX+1];
                double ans;

                if (token_type == FUNCTION)
                {
                    strcpy(fn_name, token);
                    getToken();
                    ans = eval_function(fn_name, parse_level10());
                }
                else
                {
                    ans = parse_level10();
                }
    
                return ans;
            }


/*
 * parenthesized expression or value
 */
            double parse_level10() {
                // check if it is a parenthesized expression
                if (token_type == DELIMETER)
                {
                    if (token[0] == '(' & token[1] == '\0')
                    {
                        getToken();
                        double ans = parse_level2();
                        if (token_type != DELIMETER || token[0] != ')' || token[1] || '\0')
                        {
                            throw Error(row(), col(), 3);
                        }
                        getToken();
                        return ans;
                    }
                }

                // if not parenthesized then the expression is a value
                return parse_number();
            }


            double parse_number() {
                double ans = 0;

                switch (token_type)
                {
                case NUMBER:
                    // this is a number
                    ans = strtod(token, NULL);
                    getToken();
                    break;

                case VARIABLE:
                    // this is a variable
                    ans = eval_variable(token);
                    getToken();  
                    break;
            
                default:
                    // syntax error or unexpected end of expression
                    if (token[0] == '\0')
                    {
                        throw Error(row(), col(), 6);
                    }
                    else
                    {
                        throw Error(row(), col(), 7);
                    }
                    break;
                }

                return ans;
            }


/*
 * returns the id of the given operator
 * treturns -1 if the operator is not recognized
 */
            int get_operator_id(const char op_name[]) {
                // level 2
                if (!strcmp(op_name, "&")) {return AND;}
                if (!strcmp(op_name, "|")) {return OR;}
                if (!strcmp(op_name, "<<")) {return BITSHIFTLEFT;}
                if (!strcmp(op_name, ">>")) {return BITSHIFTRIGHT;}

                // level 3
                if (!strcmp(op_name, "=")) {return EQUAL;}
                if (!strcmp(op_name, "<>")) {return UNEQUAL;}
                if (!strcmp(op_name, "<")) {return SMALLER;}
                if (!strcmp(op_name, ">")) {return LARGER;}
                if (!strcmp(op_name, "<=")) {return SMALLEREQ;}
                if (!strcmp(op_name, ">=")) {return LARGEREQ;}

                // level 4
                if (!strcmp(op_name, "+")) {return PLUS;}
                if (!strcmp(op_name, "-")) {return MINUS;}

                // level 5
                if (!strcmp(op_name, "*")) {return MULTIPLY;}
                if (!strcmp(op_name, "/")) {return DIVIDE;}
                if (!strcmp(op_name, "%")) {return MODULUS;}
                if (!strcmp(op_name, "||")) {return XOR;}

                // level 6
                if (!strcmp(op_name, "^")) {return POW;}

                // level 7
                if (!strcmp(op_name, "!")) {return FACTORIAL;}

                return -1;
            }


/*
 * evaluate an operator for given valuess
 */
            double eval_operator(const int op_id, const double &lhs, const double &rhs) {
                switch (op_id)
                {
                    // level 2
                case AND:           return static_cast<int>(lhs) & static_cast<int>(rhs);
                case OR:            return static_cast<int>(lhs) | static_cast<int>(rhs);
                case BITSHIFTLEFT:  return static_cast<int>(lhs) << static_cast<int>(rhs);
                case BITSHIFTRIGHT: return static_cast<int>(lhs) >> static_cast<int>(rhs);

                    // level 3
                case EQUAL:     return lhs == rhs;
                case UNEQUAL:   return lhs != rhs;
                case SMALLER:   return lhs < rhs;
                case LARGER:    return lhs > rhs;
                case SMALLEREQ: return lhs <= rhs;
                case LARGEREQ:  return lhs >= rhs;
        
                    // level 4
                case PLUS:      return lhs + rhs;
                case MINUS:     return lhs - rhs;
        
                    // level 5
                case MULTIPLY:  return lhs * rhs;
                case DIVIDE:    return lhs / rhs;
                case MODULUS:   return static_cast<int>(lhs) % static_cast<int>(rhs); // todo: give a warning if the values are not integer?
                case XOR:       return static_cast<int>(lhs) ^ static_cast<int>(rhs);
        
                    // level 6
                case POW:       return pow(lhs, rhs);
        
                    // level 7
                case FACTORIAL: return factorial(lhs);
                }

                throw Error(row(), col(), 104, op_id);    
                return 0;
            }


/*
 * evaluate a function
 */
            double eval_function(const char fn_name[], const double &value) {
                try
                {
                    // first make the function name upper case
                    char fnU[NAME_LEN_MAX+1];
                    toupper(fnU, fn_name);

                    // arithmetic 
                    if (!strcmp(fnU, "ABS")) {return abs(value);}
                    if (!strcmp(fnU, "EXP")) {return exp(value);}
                    if (!strcmp(fnU, "SIGN")) {return sign(value);}
                    if (!strcmp(fnU, "SQRT")) {return sqrt(value);}
                    if (!strcmp(fnU, "LOG")) {return log(value);}
                    if (!strcmp(fnU, "LOG10")) {return log10(value);}
    
                    // trigonometric
                    if (!strcmp(fnU, "SIN")) {return sin(value);}
                    if (!strcmp(fnU, "COS")) {return cos(value);}
                    if (!strcmp(fnU, "TAN")) {return tan(value);}
                    if (!strcmp(fnU, "ASIN")) {return asin(value);}
                    if (!strcmp(fnU, "ACOS")) {return acos(value);}
                    if (!strcmp(fnU, "ATAN")) {return atan(value);}
    
                    // probability
                    if (!strcmp(fnU, "FACTORIAL")) {return factorial(value);}
                }
                catch (Error err)
                {
                    // retrow error, add information about column and row of occurance
                    // TODO: doesnt work yet
                    throw Error(col(), row(), err.get_id(), err.get_msg());
                }
    
                // unknown function
                throw Error(row(), col(), 102, fn_name);
                return 0;
            }


/*
 * evaluate a variable
 */
            double eval_variable(const char var_name[]) {
                // first make the variable name uppercase
                char varU[NAME_LEN_MAX+1];
                toupper(varU, var_name);

                // check for built-in variables
                if (!strcmp(varU, "E")) {return 2.7182818284590452353602874713527;}
                if (!strcmp(varU, "PI")) {return 3.1415926535897932384626433832795;}
    
                // check for user defined variables
                double ans;
                if (user_var.get_value(var_name, &ans))
                {
                    return ans;
                }

                // unknown variable
                throw Error(row(), col(), 103, var_name);
                return 0;
            }



/*
 * Shortcut for getting the current row value (one based)
 * Returns the line of the currently handled expression
 */
            int row() {
                return -1;
            }

/*
 * Shortcut for getting the current col value (one based)
 * Returns the column (position) where the last token starts
 */
            int col() {
                return e-expr-strlen(token)+1;
            }
        };










    } // namespace expr
} // namespace sehoon

#endif // #ifndef SEHOON_EXPR_PARSER_H

