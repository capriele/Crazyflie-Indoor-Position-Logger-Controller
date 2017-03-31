import htmlPy
from math import *


class MathParser(htmlPy.Object):
    def __init__(self, app):
        super(MathParser, self).__init__()
        self.app = app
        self.addDefaultFunctions()
        self.addDefaultVariables()

    '''
        Mathematical Expression Evaluator class.
        You can set the expression member, set the functions, variables and then call
        evaluate() function that will return you the result of the mathematical expression
        given as a string.
        '''

    '''
    Mathematical expression to evaluate.
    '''
    expression = ''

    '''
    Dictionary of functions that can be used in the expression.
    '''
    functions = {'__builtins__': None};

    '''
    Dictionary of variables that can be used in the expression.
    '''
    variables = {'__builtins__': None};

    def evaluate(self):
        '''
        Evaluate the mathematical expression given as a string in the expression member variable.

        '''
        return eval(self.expression, self.variables, self.functions);

    def addDefaultFunctions(self):
        '''
        Add the following Python functions to be used in a mathemtical expression:
        acos
        asin
        atan
        atan2
        ceil
        cos
        cosh
        degrees
        exp
        fabs
        floor
        fmod
        frexp
        hypot
        ldexp
        log
        log10
        modf
        pow
        radians
        sin
        sinh
        sqrt
        tan
        tanh
        '''
        self.functions['acos'] = acos
        self.functions['asin'] = asin
        self.functions['atan'] = atan
        self.functions['atan2'] = atan2
        self.functions['ceil'] = ceil
        self.functions['cos'] = cos
        self.functions['cosh'] = cosh
        self.functions['degrees'] = degrees
        self.functions['exp'] = exp
        self.functions['fabs'] = fabs
        self.functions['floor'] = floor
        self.functions['fmod'] = fmod
        self.functions['frexp'] = frexp
        self.functions['hypot'] = hypot
        self.functions['ldexp'] = ldexp
        self.functions['log'] = log
        self.functions['log10'] = log10
        self.functions['modf'] = modf
        self.functions['pow'] = pow
        self.functions['radians'] = radians
        self.functions['sin'] = sin
        self.functions['sinh'] = sinh
        self.functions['sqrt'] = sqrt
        self.functions['tan'] = tan
        self.functions['tanh'] = tanh

    def addDefaultVariables(self):
        '''
        Add e and pi to the list of defined variables.
        '''
        self.variables['e'] = e
        self.variables['pi'] = pi

    def getVariableNames(self):
        '''
        Return a List of defined variables names in sorted order.
        '''
        mylist = list(self.variables.keys())
        try:
            mylist.remove('__builtins__')
        except ValueError:
            pass
        mylist.sort()
        return mylist

    def getFunctionNames(self):
        '''
        Return a List of defined function names in sorted order.
        '''
        mylist = list(self.functions.keys())
        try:
            mylist.remove('__builtins__')
        except ValueError:
            pass
        mylist.sort()
        return mylist

    @htmlPy.Slot(str, str, float, result=float)
    def evaluate_function(self, type, function, value):
        self.variables[type] = value
        self.expression = function
        return self.evaluate()

