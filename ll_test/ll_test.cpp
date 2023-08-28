/*
测试学习C++
*/

/*
#include <iostream> 

using namespace std; 

class Apple 
{ 
public: 
	static int i; 
	
	Apple() 
	{ 
		// Do nothing 
	}; 
}; 

int main() 
{ 
    Apple obj1; 
    Apple obj2; 
    obj1.i = 2; 
    // obj2.i = 3; 

	// 静态变量不能使用构造函数初始化
    cout << obj1.i << " "; // << obj2.i; 

} 
*/

#include<iostream> 
using namespace std; 

class Apple 
{ 
    public: 
        // static member function 
        static void printMsg() 
        {
            cout << "Welcome to Apple!" << endl;; 
        }
}; 

// main function 
int main() 
{ 
    // invoking a static member function 
    // Apple::printMsg();

    Apple a;
    a.printMsg();

} 