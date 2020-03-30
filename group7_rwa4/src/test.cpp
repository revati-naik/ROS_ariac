#include <unordered_map>
#include <string>
#include <iostream>
using namespace std;

int main(){
	unordered_map<std::string, size_t> task {};
	for (unsigned int i = 0; i < 5; ++i){
		string product_type;
		cout << "Enter product type: ";
		cin >> product_type;
		cout << "'" << product_type << "'" << " has been entered\n\n";
		task[product_type] += 1; 
	}
	for (const auto & mapItem: task){
		cout << mapItem.first << ": " << mapItem.second << endl;
	}
}