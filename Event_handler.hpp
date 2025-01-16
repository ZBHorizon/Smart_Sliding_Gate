#include <functional>
#include <list>
#include <mutex>
#include <shared_mutex>
#include <utility>
#include <cstdint>
#include <iostream> 
#include <ranges>

namespace Util {
template<typename... Parameters>
class Event {
  /*------------------------------------------------------------------------------------------------------------------*/
  /*//////// Public Interface ////////////////////////////////////////////////////////////////////////////////////////*/
  /*------------------------------------------------------------------------------------------------------------------*/
public:
  /* Constructors / Destructor */
  /*------------------------------------------------------------------------------------------------------------------*/
  Event() = default;
  Event(const Event<Parameters...>& event)
    : _functions(event._functions)
    , _functionsOnce(event._functionsOnce)
    , _masterID(event._masterID) {}
  Event(Event<Parameters...>&& event) noexcept
    : _functions(std::move(event._functions))
    , _functionsOnce(std::move(event._functionsOnce))
    , _masterID(std::move(event._masterID)) {}
  ~Event() = default;


  /* Operators */
  /*------------------------------------------------------------------------------------------------------------------*/
  Event<Parameters...>& operator=(const Event<Parameters...>& event) {
    std::scoped_lock _(_sharedMutex, event._sharedMutex);
    _functions = event._functions;
    _functionsOnce = event._functionsOnce;
    _masterID = event._masterID;
    return *this;
  }
  Event<Parameters...>& operator=(Event<Parameters...>&& event) noexcept {
    std::scoped_lock _(_sharedMutex, event._sharedMutex);
    std::swap(_functions, event._functions);
    std::swap(_functionsOnce, event._functionsOnce);
    std::swap(_masterID, event._masterID);
    return *this;
  }


  /* Controller */
  /*------------------------------------------------------------------------------------------------------------------*/
  void Clear() {
    std::scoped_lock _(_sharedMutex);
    _functions.clear();
    _functionsOnce.clear();
  }


  /* Add Listener */
  /*------------------------------------------------------------------------------------------------------------------*/
  //! Adds a normal listening class method which is called every time the event is triggered.
  template<class C, typename Fn>
  uint64_t AddListener(C* instance, Fn function) {
    return (instance && function) ? AddListener(std::bind_front(function, instance)) : 0;
  }

  //! Adds a listening function which is called every time the event is triggered.
  uint64_t AddListener(const std::function<void(Parameters...)>& function) {
    std::scoped_lock _(_sharedMutex);
    return _functions.emplace_back(std::make_pair(++_masterID, function)).first;
  }

  //! Adds a listening class method which is called only once if the event is triggered.\n
  //! The function is removed internally even before it is actually called.\n
  //! Thus the function can add itself again if required.
  template<class C, typename Fn>
  uint64_t AddListenerForOnce(C* instance, Fn function) {
    return (instance && function) ? AddListenerForOnce(std::bind_front(function, instance)) : 0;
  }

  //! Adds a listening function which is called only once if the event is triggered.\n
  //! The function is removed internally even before it is actually called.\n
  //! Thus the function can add itself again if required.
  template<typename Fn>
  uint64_t AddListenerForOnce(const std::function<void(Parameters...)>& function) {
    std::scoped_lock _(_sharedMutex);
    return _functionsOnce.emplace_back(std::make_pair(++_masterID, function)).first;
  }


  /* Trigger */
  /*------------------------------------------------------------------------------------------------------------------*/
  //! Triggeres listeners.\n
  //! The once-listeners are removed internally even before they are actually called.
  virtual void Trigger(Parameters... parameters) {
    std::shared_lock lock(_sharedMutex);
    if (_functions.empty() && _functionsOnce.empty()) return;

    auto functions = { _functions, _functionsOnce };
    _functionsOnce.clear();
    lock.unlock();

    for (const auto& [_, function] : functions | std::views::join) {
        function(parameters...);
    }
  }


  /* Remove Listener */
  /*------------------------------------------------------------------------------------------------------------------*/
  bool RemoveListener(uint64_t id) {
    bool result = false;
    _functions.remove_if([&](const auto& pair) { return (pair.first == id) ? result = true : false; });
    _functionsOnce.remove_if([&](const auto& pair) { return (pair.first == id) ? result = true : false; });
    return result;
  }


  /* Getter / Setter */
  /*------------------------------------------------------------------------------------------------------------------*/
  //! Checks for existing listeners.
  bool IsEmpty() const {
    std::shared_lock _(_sharedMutex);
    return (_functions.empty() && _functionsOnce.empty());
  }


  /*------------------------------------------------------------------------------------------------------------------*/
  /*//////// Private Interface ///////////////////////////////////////////////////////////////////////////////////////*/
  /*------------------------------------------------------------------------------------------------------------------*/
private:
  /* Variables */
  /*------------------------------------------------------------------------------------------------------------------*/
  mutable std::shared_mutex _sharedMutex;

  std::list<std::pair<uint64_t, std::function<void(Parameters...)>>> _functions;
  std::list<std::pair<uint64_t, std::function<void(Parameters...)>>> _functionsOnce;

  uint64_t _masterID = 0;
};
}
