# C++ Naming Conventions: Comprehensive Guide

Copy from :
https://medium.com/@ryan_forrester_/c-naming-conventions-comprehensive-guide-6b813c17c23b

## File Names
C++ projects typically involve two types of files: header files and source files.
- Header files: Use the `.h` or `.hpp` extension
- Source files: Use the `.cpp` extension

Example:
```
my_class.h
my_class.cpp
```
Some projects use uppercase for header guards:
```
// In my_class.h
#ifndef MY_CLASS_H
#define MY_CLASS_H

// Class definition here

#endif // MY_CLASS_H
```

## Namespaces
Namespaces use lowercase with underscores for word separation:
```
namespace my_project {
namespace sub_system {

// Code here

} // namespace sub_system
} // namespace my_project
```
Avoid using `using namespace` in header files to prevent naming conflicts.

## Classes and Structs
Use PascalCase for class and struct names:
```
class MyClass {
    // Class members
};

struct MyStruct {
    // Struct members
};
```

## Functions and Methods
Use camelCase for function and method names:
```
void myFunction();

class MyClass {
public:
    void myMethod();
};
```

## Variables
Local variables and function parameters use camelCase:
```
void processData(int inputValue) {
    int localVariable = inputValue * 2;
    // More code
}
```
Google C++ Style Guide, Local variables use snake_case :
```
void process_data(int input_value) 
{
    int local_variable = input_value * 2;
    // More code
}
```

## Member Variables
There are two common conventions for member variables:
- Underscore suffix:
```
class MyClass {
private:
    int value_;
    std::string name_;
};
```
- m_prefix:
```
class MyClass {
private:
    int m_value;
    std::string m_name;
};
```
Choose one and stick to it consistently throughout your project.

## Constants and Enums
Use PascalCase for enum names and UPPER_CASE for enum values and constants:

```
enum class Color {
    RED,
    GREEN,
    BLUE
};

const int MAX_SIZE = 100;
constexpr double PI = 3.14159265358979323846;
```

## Templates
Use PascalCase for template parameters:
```
template<typename InputIterator, typename T>
InputIterator find(InputIterator First, InputIterator Last, const T& Value);
```

## Macros
Use UPPER_CASE for macro names:
```
#define MAX(a, b) ((a) > (b) ? (a) : (b))
```

However, remember that in modern C++, it’s often better to use `constexpr` functions instead of macros.

## Hungarian Notation: A Historical Note
Hungarian notation, which prefixes variable names with type information (e.g., `iCount` for an integer count), was once popular but is now generally discouraged in C++. Modern IDEs and strong typing make this notation redundant and can clutter code.

## Practical Examples
Let’s look at a more comprehensive example that ties these conventions together:

- shape_processor.h
```
// file: shape_processor.h
#ifndef SHAPE_PROCESSOR_H
#define SHAPE_PROCESSOR_H

#include <vector>
#include <string>

namespace geometry {

enum class ShapeType {
    CIRCLE,
    SQUARE,
    TRIANGLE
};

class Shape {
public:
    virtual double calculateArea() const = 0;
    virtual std::string getName() const = 0;
    virtual ~Shape() = default;
};

class Circle : public Shape {
public:
    explicit Circle(double radius);
    double calculateArea() const override;
    std::string getName() const override;

private:
    double radius_;
};

class ShapeProcessor {
public:
    void addShape(std::unique_ptr<Shape> shape);
    double getTotalArea() const;
    std::vector<std::string> getShapeNames() const;

private:
    std::vector<std::unique_ptr<Shape>> shapes_;
};

} // namespace geometry

#endif // SHAPE_PROCESSOR_H
```
- shape_processor.cpp

```
// file: shape_processor.cpp
#include "shape_processor.h"
#include <cmath>

namespace geometry {

Circle::Circle(double radius) : radius_(radius) {}

double Circle::calculateArea() const {
    constexpr double PI = 3.14159265358979323846;
    return PI * radius_ * radius_;
}

std::string Circle::getName() const {
    return "Circle";
}

void ShapeProcessor::addShape(std::unique_ptr<Shape> shape) {
    shapes_.push_back(std::move(shape));
}

double ShapeProcessor::getTotalArea() const {
    double totalArea = 0.0;
    for (const auto& shape : shapes_) {
        totalArea += shape->calculateArea();
    }
    return totalArea;
}

std::vector<std::string> ShapeProcessor::getShapeNames() const {
    std::vector<std::string> names;
    names.reserve(shapes_.size());
    for (const auto& shape : shapes_) {
        names.push_back(shape->getName());
    }
    return names;
}

} // namespace geometry
```

This example demonstrates various naming conventions:
- Namespace `geometry` in lowercase
- Classes `Shape`, `Circle`, and `ShapeProcessor` in PascalCase
- Methods like `calculateArea` and `getShapeNames` in camelCase
- Member variable `radius_` with an underscore suffix
- Enum class `ShapeType` in PascalCase with UPPER_CASE values
- Constant `PI` in UPPER_CASE

## Real-World Considerations

While these conventions are widely used, it’s important to note that different organizations or open-source projects may have their own specific guidelines. When working on an existing project, always follow its established conventions for consistency.

## Google C++ Style Guide
Google’s C++ Style Guide, for instance, has some differences:
- They use snake_case for function names and variable names
- Member variables have a trailing underscore
- Getters don’t have a `get` prefix unless it does significant work

Example based on Google’s style:
```
class TableInfo {
public:
    void set_name(const string& name) { name_ = name; }
    string name() const { return name_; }
    
    void set_num_entries(int num_entries) { num_entries_ = num_entries; }
    int num_entries() const { return num_entries_; }

private:
    string name_;
    int num_entries_;
};
```

## C# Style
There are following three terminologies are used to declare C# and .NET naming standards. 
- Camel Case (camelCase): In this standard, the first letter of the word always in small letter and after that each word starts with a capital letter.
- Pascal Case (PascalCase): In this the first letter of every word is in capital letter.
- Underscore Prefix (_underScore): For underscore ( _ ), the word after _ use camelCase terminology.

| Feature         | Format          |
| --------------- | --------------- |
| Private field	  | _camelCase  |
| Public field	  | PascalCase  |
| Protected field | PascalCase  |
| Internal field  | PascalCase  |
| Property        | PascalCase  |
| Methode         | PascalCase  |
| Class           | PascalCase  |
| Interface       | IPascalCase |
| Local variable  | camelCase   |
| Parameter       | camelCase   |

## Boost Library
The Boost C++ libraries, on the other hand, use:
- Underscore-separated lowercase for most names
- PascalCase for template parameters

Example based on Boost style:
```
template <class InputIterator, class Distance>
void advance(InputIterator& i, Distance n);
```

## Naming for Readability and Maintenance
Regardless of the specific style you choose, the primary goal of naming conventions is to enhance code readability and maintainability. Here are some general tips:

1. Be Descriptive: Choose names that clearly describe the purpose or content of the entity.
```
// Poor
int x;

// Better
int numberOfStudents;
```

2. Avoid Abbreviations: Unless they’re widely understood, spell out words fully.
```
// Poor
int custCnt;

// Better
int customerCount;
```
3. Use Meaningful Distinctions: Avoid names that differ only slightly.
```
// Poor
void processData(const std::vector<int>& data);
void processInformation(const std::vector<int>& information);

// Better
void processRawData(const std::vector<int>& data);
void processSummaryData(const std::vector<int>& summaryData);
```

4. Be Consistent: Once you choose a convention, stick to it throughout your project.

## Conclusion
While the specific conventions may vary between projects or organizations, the underlying principles remain the same: be clear, be consistent, and prioritize readability.

Remember, the best naming convention is the one that your team agrees on and consistently applies. It’s worth taking the time to establish and document these conventions at the start of a project, as they can significantly reduce cognitive load and improve collaboration in the long run.