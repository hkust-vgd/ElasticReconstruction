#pragma once

#include <vector>
#include <iostream>
#include <map>
#include <fstream>

BEGIN_INCREON_SPACE

template <typename T>
static inline bool isNan(T x) { return x != x; }

template <typename T>
inline std::string toString(const T & t) {
   std::ostringstream os;
   os << t;
   return os.str();
}

inline void parseIni(std::string fn, std::map<std::string, std::string> &param) {
	std::ifstream ifile;
	ifile.open(fn.c_str(), std::ios::in);

	char line[256];
	while(ifile.getline(line,256)) {
		if (strlen(line) == 0 || line[0] == '\0' || line[0] == '#')
		  continue;

		std::string s1, s2;
		s1 = strtok(line, " \t\r\n");
		s2 = strtok(NULL, " \t\r\n");

		param.insert(std::make_pair(s1, s2));
	}

	ifile.close();
}

template <typename ObserverType>
class Observable {
public:
    void attachObserver(ObserverType *ob) {
        for (int i = 0; i < observers.size(); ++i) {
            if (observers[i] == ob) {
                return;
            }
        }
        observers.push_back(ob);
    }

    void detachObserver(ObserverType *ob) {
        /*
        // this code does not compile on OS X
        for (ObserverTypes::iterator i = observers.begin(); i != observers.end(); ++i) {
            if ((*i) == ob) {
                observers.erase(i);
                break;
            }
        }*/
        for (int i = 0; i < observers.size(); ++i) {
            if (observers[i] == ob) {
                observers.erase(observers.begin() + i);
                break;
            }
        }   
    }

    void clear() {
        observers.clear();
    }

    int countObservers() {
        return observers.size();
    }

protected:
    std::vector<ObserverType*> observers;
};

END_INCREON_SPACE
