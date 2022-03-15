#include <iostream>
#include <sstream>
#include <fstream>
#include <limits.h>
#include "archive.h"

////////////////////////////////////////////////////
// Test/usage examples for archive
////////////////////////////////////////////////////


struct Message
{
    int id;
    std::string name;
    std::vector<std::string> groups;
    float coolNumbers[3];


    Message(int nid = 1, const std::string& nname = "unknown") : id(nid), name(nname)
    {
        coolNumbers[0] = 3.1416f;
        coolNumbers[1] = 2.7182f;
        coolNumbers[2] = 1.6180f;

        groups.push_back("wheel");
        groups.push_back("lp");
        groups.push_back("root");
        groups.push_back("sudo");
    }

    template <class T>
        void Serialize(T& archive)
        {
            archive & id & name & groups & coolNumbers;
        }

    // equality operator for the tests
    bool operator==(const Message& m) const
    {
        if(id != m.id)
            return false;
        if(name != m.name)
            return false;
        for(int i = 0; i < 3; ++i)
            if(coolNumbers[i] != m.coolNumbers[i])
                return false;
        for(unsigned int i = 0; i < groups.size(); ++i)
            if(groups[i] != m.groups[i])
                return false;

        return true;
    }
    bool operator!=(const Message& m) const
    {
        return !(*this == m);
    }
};

    template <class T>
void Test(const T& testValue)
{
    std::stringstream s;
    Archive<std::stringstream> a(s);
    T value(testValue);

    a << value;
    assert(testValue == value);

    value = T();
    assert(testValue != value);

    a >> value;
    assert(testValue == value);
}

    template <class T, size_t N>
void TestArray(T (&value)[N])
{
    std::stringstream s;
    Archive<std::stringstream> a(s);

    a << value;
    a >> value;
}

void TestFile()
{
    // Serialize to file
    std::ofstream ofile("test.bin", std::ios::binary);
    Archive<std::ofstream> a(ofile);
    int i = 123;
    std::string name = "mouton";
    float pi = 3.1416f;
    Message m(666, "the beast");

    a << i << name << pi << m;
    ofile.close();


    // Unserialize from file:
    int i2;
    std::string name2;
    float pi2;
    Message m2;

    std::ifstream ifile("test.bin", std::ios::binary);
    if(!ifile.is_open())
    {
        std::cout << "ERROR file doesn't exist.." << std::endl;
        return;
    }

    Archive<std::ifstream> a2(ifile);
    a2 >> i2 >> name2 >> pi2 >> m2;
    ifile.close();

    assert(i == i2);
    assert(name == name2);
    assert(pi == pi2);
    assert(m == m2);
}


int main()
{
    // POD
    std::cout << "POD..." << std::endl;
    Test(true);
    
    Test((char)123);
    Test((char)-123);
    Test((char)SCHAR_MIN);
    Test((char)SCHAR_MAX);
    Test((unsigned char)123);
    Test((unsigned char)UCHAR_MAX);

    Test((short)123);
    Test((short)-123);
    Test((short)SHRT_MIN);
    Test((short)SHRT_MAX);
    Test((unsigned short)123);

    Test((int)123);
    Test((int)-123);
    Test((int)INT_MIN);
    Test((int)INT_MAX);
    Test((unsigned int)123);
    Test((unsigned int)UINT_MAX);

    Test((long)123);
    Test((long)-123);
    Test((long)LONG_MIN);
    Test((long)LONG_MAX);
    Test((unsigned long)123);
    Test((unsigned long)ULONG_MAX);

    Test((long long)123);
    Test((long long)-123);
    Test((long long)LLONG_MIN);
    Test((long long)LLONG_MAX);
    Test((unsigned long long)123);
    Test((unsigned long long)ULLONG_MAX);

    Test(std::string("salut"));
    Test(123.456f);
    Test(-123.456f);
    Test(123.456);
    Test(-123.456);

    // Test problem with NaN (reported by @NikishinRoman on vstudio: https://github.com/voidah/archive/issues/10)
    Test(-0.348907441f);

    // Array
    std::cout << "Array..." << std::endl;
    int ii[] = {1, 2, 3, 4, 5, 6};
    TestArray(ii);

    // STL
    std::cout << "STL..." << std::endl;
    Test(std::vector<int>({1, 2, 3, 4, 5}));
    Test(std::vector<std::string>({"a", "bb", "ccc", "dddd"}));
    Test(std::deque<int>({1, 2, 3, 4, 5}));
    Test(std::deque<std::string>({"a", "bb", "ccc", "dddd"}));
    Test(std::list<int>({1, 2, 3, 4, 5}));
    Test(std::list<std::string>({"a", "bb", "ccc", "dddd"}));
    Test(std::set<int>({1, 2, 3, 4, 5}));
    Test(std::set<std::string>({"a", "bb", "ccc", "dddd"}));
    Test(std::multiset<int>({1, 2, 3, 4, 5}));
    Test(std::multiset<std::string>({"a", "bb", "ccc", "dddd"}));
    Test(std::map<int, std::string>({std::make_pair(1, "a"), std::make_pair(2, "bb")}));
    Test(std::multimap<int, std::string>({std::make_pair(1, "a"), std::make_pair(2, "bb")}));

    // User type
    std::cout << "User type..." << std::endl;
    Test(Message(666, "the beast"));

    // Serialize to/from file
    std::cout << "In/Out from file..." << std::endl;
    TestFile();

    // invalid data, invalid string length
    {
        std::cout << "Testing invalid data (invalid string length), should throw an exception" << std::endl;
        std::stringstream s1;
        Archive<std::stringstream> a1(s1);
        a1 << std::string("salut");

        std::string data = s1.str();

        // Corrupt string length:
        data[0] = (char)255; // the first 4 bytes are the string length
        data[1] = (char)255; // the first 4 bytes are the string length
        data[2] = (char)255; // the first 4 bytes are the string length
        data[3] = (char)255; // the first 4 bytes are the string length

        // Try to read back the data:
        try
        {
            std::istringstream s2(data);
            Archive<std::istringstream> a2(s2);
            std::string value;
            a2 >> value;
        }
        catch(...)
        {
            std::cout << "    GOOD! exception catched" << std::endl;
        }
    }

    // invalid data, not enough data
    {
        std::cout << "Testing invalid data (not enough data), should throw an exception" << std::endl;
        std::stringstream s1;
        Archive<std::stringstream> a1(s1);
        a1 << 'a';

        std::string data = s1.str();
        std::cout << data << std::endl;

        // Try to read back the data:
        try
        {
            std::istringstream s2(data);
            Archive<std::istringstream> a2(s2);
            int value;
            a2 >> value;
        }
        catch(...)
        {
            std::cout << "    GOOD! exception catched" << std::endl;
        }
    }
}
