#pragma once
#include <cerrno>
#include <source_location>
#include <set>
#include <unordered_map>
#include <iostream>

// #if defined(WIN32)
// #  include <Windows.h>
// #undef ERROR
// #undef INPUT
// #undef ms

// #endif



namespace SlidingGate{
class Log {
  /*------------------------------------------------------------------------------------------------------------------*/
  /*//////// Public Interface ////////////////////////////////////////////////////////////////////////////////////////*/
  /*------------------------------------------------------------------------------------------------------------------*/
public:
  /* Types */
  /*------------------------------------------------------------------------------------------------------------------*/
  enum class Level : uint8_t { NONE = 0, ERROR = 1, WARNING = 2, DEBUG = 4, TRACE = 8, INFO = 16, ALL = 31 };

  struct Prefix {
    Prefix(Level level, std::string tag, std::string subTag, std::source_location location)
      : level(level)
      , location(std::move(location))
      , tag(std::move(tag))
      , subTag(std::move(subTag)) {}

    Level level = Level::NONE;
    std::chrono::system_clock::time_point timePoint = std::chrono::system_clock::now();
    std::source_location location;
    std::string tag;
    std::string subTag;
  };

  class Exception : public std::exception {
  public:
    Exception(std::string message)
      : _message(std::move(message)) {}

    const char* what() const noexcept override { return _message.c_str(); }

    const std::string _message;
  };

  class Handler {
  public:
    virtual void Write(const Prefix& prefix, const std::string& str) = 0;
  };

  class File;
  class Stream;


  /* Constructors / Destructor */
  /*------------------------------------------------------------------------------------------------------------------*/
  Log(Prefix prefix)
    : _prefix(std::move(prefix))
    // ERROR always enables
    , _isEnabled(_prefix.tag.empty() || ContainsActiveTag(_prefix.tag, _prefix.level) || _prefix.level == Level::ERROR) {}
  Log(const Log&) = delete;
  Log(Log&&) = default;
  ~Log() { Flush(); }


  /* Operators */
  /*------------------------------------------------------------------------------------------------------------------*/
  Log& operator=(const Log&) = delete;
  Log& operator=(Log&&) = default;

  template<typename T>
  Log& operator<<(const T& value) {
    if (_isEnabled) _ss << value;
    return *this;
  }
  Log& operator<<(std::ostream& (*fn)(std::ostream&)) {
    if (_isEnabled) (*fn)(_ss);
    return *this;
  }
  Log& operator<<(Log& (Log::*fn)()) { return (this->*fn)(); }
  void operator<<(void (Log::*fn)()) { (this->*fn)(); }


  /* Controller */
  /*------------------------------------------------------------------------------------------------------------------*/
  static inline void AddHandler(const std::shared_ptr<Handler>& handler) { GetHandlers().insert(handler); }
  static inline void RemoveHandler(const std::shared_ptr<Handler>& handler) { GetHandlers().erase(handler); }
  static inline void ClearHandlers() { GetHandlers().clear(); }

  Log& EndLine() {
    if (!_isEnabled) return *this;

    _ss << '\n';
    return *this;
  }

  //! Pass data to Handlers
  void Flush() {
    if (!_isEnabled) return;

    _ss << '\n';
    auto str = _ss.str();
    _ss.str("");

    for (const auto& handler : GetHandlers()) {
      try {
        handler->Write(_prefix, str);
      }
      catch (const std::exception& ex) {
        std::cout << "Log::Flush(): " << ex.what() << '\n' << std::flush;
      }
      catch (...) {
      }
    }
  }

  //! Flushes and throws the logged data
  [[noreturn]] void Throw() {
    _isEnabled = true;
    auto str = _ss.str();
    Flush();
    _isEnabled = false;
    throw Exception(str);
  }

  //! Enables/Disables specific tag that can be logged to
  static void ConfigureTag(const std::string& tag, bool isActive, Level level = Level::ALL) {
    if (isActive) {
      (uint8_t&)GetActiveTags()[tag] |= std::uint8_t(level);
    } else {
      (uint8_t&)GetActiveTags()[tag] &= ~std::uint8_t(level);
    }
  }
  static void EnableTag(const std::string& tag, Level level = Level::ALL) { ConfigureTag(tag, true, level); }
  static void DisableTag(const std::string& tag, Level level = Level::ALL) { ConfigureTag(tag, false, level); }

  //! \param level Any of those levels must be active
  static bool ContainsActiveTag(const std::string& tag, Level level = Level::ALL) {
    auto iter = GetActiveTags().find(tag);
    return (iter != GetActiveTags().end() && !!((uint8_t)iter->second & (uint8_t)level)); // any level
  }

  static std::string StringifySystemError(int errorCode) {
    std::stringstream ss;
    ss << errorCode << " - " << std::strerror(errorCode); // NOLINT
    return ss.str();
  }


  /* Getter / Setter */
  /*------------------------------------------------------------------------------------------------------------------*/
  static std::set<std::shared_ptr<Handler>>& GetHandlers() {
    static std::set<std::shared_ptr<Handler>> result;
    return result;
  }

  static std::unordered_map<std::string, Level>& GetActiveTags() {
    static std::unordered_map<std::string, Level> result;
    return result;
  }

  inline std::string GetString() const { return _ss.str(); }

  static int GetSystemError() {
#if IS_WINDOWS
    return ::GetLastError();
#else
    return errno;
#endif
  }

  inline bool IsEnabled() const { return _isEnabled; }
  inline void SetEnabled(bool value) { _isEnabled = value; }


  /* Constants */
  /*------------------------------------------------------------------------------------------------------------------*/
  static constexpr const char* PRINT_RED { "\x001b[0;31m" };
  static constexpr const char* PRINT_GREEN { "\x001b[0;32m" };
  static constexpr const char* PRINT_YELLOW { "\x001b[0;33m" };
  static constexpr const char* PRINT_BLUE { "\x001b[0;34m" };
  static constexpr const char* PRINT_PURPLE { "\x001b[0;35m" };
  static constexpr const char* PRINT_TURQUOISE { "\x001b[0;36m" };

  static constexpr const char* PRINT_RED_BOLD { "\x001b[1;31m" };
  static constexpr const char* PRINT_GREEN_BOLD { "\x001b[1;32m" };
  static constexpr const char* PRINT_YELLOW_BOLD { "\x001b[1;33m" };
  static constexpr const char* PRINT_BLUE_BOLD { "\x001b[1;34m" };
  static constexpr const char* PRINT_PURPLE_BOLD { "\x001b[1;35m" };
  static constexpr const char* PRINT_TURQUOISE_BOLD { "\x001b[1;36m" };

  static constexpr const char* PRINT_RESET { "\x001b[0m" };


  /*------------------------------------------------------------------------------------------------------------------*/
  /*//////// Private Interface ///////////////////////////////////////////////////////////////////////////////////////*/
  /*------------------------------------------------------------------------------------------------------------------*/
private:
  /* Variables */
  /*------------------------------------------------------------------------------------------------------------------*/
  std::stringstream _ss;

  Prefix _prefix;

  bool _isEnabled = false;
};


/*----------------------------------------------------------------------------------------------------------------------*/
/*//////// Global Interface ////////////////////////////////////////////////////////////////////////////////////////////*/
/*----------------------------------------------------------------------------------------------------------------------*/
/* Operators */
/*----------------------------------------------------------------------------------------------------------------------*/
template<typename T>
inline const std::shared_ptr<Log>& operator<<(const std::shared_ptr<Log>& log, const T& value) {
  *log << value;
  return log;
}
inline const std::shared_ptr<Log>& operator<<(const std::shared_ptr<Log>& log, std::ostream& (*fn)(std::ostream&)) {
  *log << fn;
  return log;
}

inline bool operator!(Log::Level a) { return !(uint8_t)a; }

inline Log::Level operator&(Log::Level a, Log::Level b) { return Log::Level((uint8_t)a & (uint8_t)b); }
inline Log::Level& operator&=(Log::Level& a, Log::Level b) { return (a = Log::Level((uint8_t)a & (uint8_t)b)); }

inline Log::Level operator|(Log::Level a, Log::Level b) { return Log::Level((uint8_t)a | (uint8_t)b); }
inline Log::Level& operator|=(Log::Level& a, Log::Level b) { return (a = Log::Level((uint8_t)a | (uint8_t)b)); }

inline Log::Level operator~(Log::Level a) { return Log::Level(~(uint8_t)a); }
}


/* Controller */
/*----------------------------------------------------------------------------------------------------------------------*/
inline std::shared_ptr<SlidingGate::Log> LOG_CREATE_DEFAULT(SlidingGate::Log::Level level, const std::string& tag, const std::string& subTag,
  const std::source_location location) {
  return std::make_shared<SlidingGate::Log>(SlidingGate::Log::Prefix(level, tag, subTag, location));
}

// Default
inline auto LOG_ERROR(const std::string& tag = "", const std::string& subTag = "", const std::source_location location = std::source_location::current()) {
  return LOG_CREATE_DEFAULT(SlidingGate::Log::Level::ERROR, tag, subTag, location);
}
inline auto LOG_WARNING(const std::string& tag = "", const std::string& subTag = "", const std::source_location location = std::source_location::current()) {
  return LOG_CREATE_DEFAULT(SlidingGate::Log::Level::WARNING, tag, subTag, location);
}
inline auto LOG_DEBUG(const std::string& tag = "", const std::string& subTag = "", const std::source_location location = std::source_location::current()) {
  return LOG_CREATE_DEFAULT(SlidingGate::Log::Level::DEBUG, tag, subTag, location);
}
inline auto LOG_TRACE(const std::string& tag = "", const std::string& subTag = "", const std::source_location location = std::source_location::current()) {
  return LOG_CREATE_DEFAULT(SlidingGate::Log::Level::TRACE, tag, subTag, location);
}
inline auto LOG_INFO(const std::string& tag = "", const std::string& subTag = "", const std::source_location location = std::source_location::current()) {
  return LOG_CREATE_DEFAULT(SlidingGate::Log::Level::INFO, tag, subTag, location);
}

// System Error
inline auto LOG_SYSTEM_ERROR(const std::string& tag = "", const std::string& subTag = "",
  const std::source_location location = std::source_location::current()) {
  auto errorCode = SlidingGate::Log::GetSystemError();
  return LOG_ERROR(tag, subTag, location) << SlidingGate::Log::StringifySystemError(errorCode);
}
inline auto LOG_SYSTEM_WARNING(const std::string& tag = "", const std::string& subTag = "",
  const std::source_location location = std::source_location::current()) {
  auto errorCode = SlidingGate::Log::GetSystemError();
  return LOG_WARNING(tag, subTag, location) << SlidingGate::Log::StringifySystemError(errorCode);
}
inline auto LOG_SYSTEM_DEBUG(const std::string& tag = "", const std::string& subTag = "",
  const std::source_location location = std::source_location::current()) {
  auto errorCode = SlidingGate::Log::GetSystemError();
  return LOG_DEBUG(tag, subTag, location) << SlidingGate::Log::StringifySystemError(errorCode);
}
inline auto LOG_SYSTEM_TRACE(const std::string& tag = "", const std::string& subTag = "",
  const std::source_location location = std::source_location::current()) {
  auto errorCode = SlidingGate::Log::GetSystemError();
  return LOG_TRACE(tag, subTag, location) << SlidingGate::Log::StringifySystemError(errorCode);
}
inline auto LOG_SYSTEM_INFO(const std::string& tag = "", const std::string& subTag = "",
  const std::source_location location = std::source_location::current()) {
  auto errorCode = SlidingGate::Log::GetSystemError();
  return LOG_INFO(tag, subTag, location) << SlidingGate::Log::StringifySystemError(errorCode);
}

// Conditional
inline std::shared_ptr<SlidingGate::Log> LOG_CREATE_DEFAULT_IF(bool isActive, SlidingGate::Log::Level level, const std::string& tag,
  const std::string& subTag, const std::source_location location) {
  auto log = std::make_shared<SlidingGate::Log>(SlidingGate::Log::Prefix(level, tag, subTag, location));
  log->SetEnabled(isActive && log->IsEnabled());
  return log;
}

inline auto LOG_ERROR_IF(bool isActive, const std::string& tag = "", const std::string& subTag = "",
  const std::source_location location = std::source_location::current()) {
  return LOG_CREATE_DEFAULT_IF(isActive, SlidingGate::Log::Level::ERROR, tag, subTag, location);
}
inline auto LOG_WARNING_IF(bool isActive, const std::string& tag = "", const std::string& subTag = "",
  const std::source_location location = std::source_location::current()) {
  return LOG_CREATE_DEFAULT_IF(isActive, SlidingGate::Log::Level::WARNING, tag, subTag, location);
}
inline auto LOG_DEBUG_IF(bool isActive, const std::string& tag = "", const std::string& subTag = "",
  const std::source_location location = std::source_location::current()) {
  return LOG_CREATE_DEFAULT_IF(isActive, SlidingGate::Log::Level::DEBUG, tag, subTag, location);
}
inline auto LOG_TRACE_IF(bool isActive, const std::string& tag = "", const std::string& subTag = "",
  const std::source_location location = std::source_location::current()) {
  return LOG_CREATE_DEFAULT_IF(isActive, SlidingGate::Log::Level::TRACE, tag, subTag, location);
}
inline auto LOG_INFO_IF(bool isActive, const std::string& tag = "", const std::string& subTag = "",
  const std::source_location location = std::source_location::current()) {
  return LOG_CREATE_DEFAULT_IF(isActive, SlidingGate::Log::Level::INFO, tag, subTag, location);
}


// Occasional
// #define LOG_CONCAT(x, y) x##y
//
// #define LOG_NTH(n, line)                                  \
//   static size LOG_CONCAT(LOG_NTH_, line) = n;         \
//   static size LOG_CONCAT(LOG_NTH_COUNTER_, line) = 0; \
//   if ((++LOG_CONCAT(LOG_NTH_COUNTER_, line) % LOG_CONCAT(LOG_NTH_, line)) == 0)
//
// #define LOG_ERROR_NTH(n) LOG_NTH(n, __LINE__) LOG_ERROR
// #define LOG_WARNING_NTH(n) LOG_NTH(n, __LINE__) LOG_WARNING
// #define LOG_DEBUG_NTH(n) LOG_NTH(n, __LINE__) LOG_DEBUG
// #define LOG_TRACE_NTH(n) LOG_NTH(n, __LINE__) LOG_TRACE
// #define LOG_INFO_NTH(n) LOG_NTH(n, __LINE__) LOG_INFO


/* Constants */
/*----------------------------------------------------------------------------------------------------------------------*/
constexpr std::string_view LOG_LINE_25 { "-------------------------" };
constexpr std::string_view LOG_LINE_50 { "--------------------------------------------------" };
constexpr std::string_view LOG_LINE_75 { "---------------------------------------------------------------------------" };
constexpr std::string_view LOG_LINE_100 { "----------------------------------------------------------------------------------------------------" };

constexpr auto LOG_ENDL = &SlidingGate::Log::EndLine;
constexpr auto LOG_FLUSH = &SlidingGate::Log::Flush;
constexpr auto LOG_THROW = &SlidingGate::Log::Throw;
