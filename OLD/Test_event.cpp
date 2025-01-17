#include <Event_handler.hpp>

Util::Event<int /* pin */, int /* mode */> onChange;
void listener(int a, int b) {
	std::cout << "Listener called " << a << " " << b << " \n ";
}
int main(){
	uint64_t id = onChange.AddListener(listener);
	onChange.Trigger(1, 0);
	bool success = onChange.RemoveListener(id);
	std::cout << "Listener removed: " << success << "\n";
	onChange.Trigger(1, 1);
	return 0;
}	

