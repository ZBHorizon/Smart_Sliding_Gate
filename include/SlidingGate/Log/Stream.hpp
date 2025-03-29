#include <SlidingGate/Log.hpp>

#include <regex>
#include <filesystem>

namespace SlidingGate{
class Log::Stream : public Log::Handler {
  /*------------------------------------------------------------------------------------------------------------------*/
  /*//////// Public Interface ////////////////////////////////////////////////////////////////////////////////////////*/
  /*------------------------------------------------------------------------------------------------------------------*/
public:
  /* Constructors / Destructor */
  /*------------------------------------------------------------------------------------------------------------------*/
  Stream(std::ostream* ostream, Level enabledLevels = Level::ALL)
    : _ostream(ostream)
    , _enabledLevels(enabledLevels) {}
  Stream(const Stream&) = delete;
  Stream(Stream&&) = delete;
  ~Stream() = default;


  /* Controller */
  /*------------------------------------------------------------------------------------------------------------------*/
  void Write(const Prefix& prefix, const std::string& str) override {
    if (!(prefix.level & _enabledLevels)) return;
    if (str.empty()) return;

    const std::scoped_lock _(_recursiveMutex);
    if (!_isEnabled) return;
    if (!prefix.tag.empty() && !_activeTags.empty() && !ContainsActiveTag(prefix.tag, prefix.level)) return;

    if (_isColorized) WriteColorStart(prefix.level);

    if (_isLevelWritten) WriteLevel(prefix.level);
    if (_isTimestampWritten) WriteTimestamp(prefix.timePoint);
    if (_isLocationWritten) WriteLocation(prefix.location);
    if (_isTagWritten && !prefix.tag.empty()) _ss << "[" << prefix.tag << "]";
    if (_isSubTagWritten && !prefix.subTag.empty()) _ss << "[" << prefix.subTag << "]";
    if (_isFunctionNameWritten) WriteFunctionName(prefix.location);

    if (_isColorized) WriteColorEnd();

    if (_ss.tellp() > 0) _ss << " ";
    _ss << str;
    Write(_ss.str());
    _ss.str("");
  }

  //! Enables/Disables specific tag that can be logged to
  void ConfigureTag(const std::string& tag, bool isActive, Level level = Level::ALL) {
    const std::scoped_lock _(_recursiveMutex);
    if (isActive) {
      _activeTags[tag] |= level;
    } else {
      _activeTags[tag] &= ~level;
    }
  }

  //! \param level Any of those levels must be active
  bool ContainsActiveTag(const std::string& tag, Level level = Level::ALL) {
    const std::scoped_lock _(_recursiveMutex);

    auto iter = _activeTags.find(tag);
    return (iter != _activeTags.end() && !!(iter->second & level)); // any level
  }

  inline void DisablePrefix() {
    _isColorized = false;
    _isLevelWritten = false;
    _isTagWritten = false;
    _isSubTagWritten = false;
    _isFunctionNameWritten = false;
    _isLocationWritten = false;
    _isTimestampWritten = false;
  }


  /* Getter / Setter */
  /*------------------------------------------------------------------------------------------------------------------*/
  inline std::recursive_mutex& GetRecursiveMutex() { return _recursiveMutex; }

  inline std::unordered_map<std::string, Level> GetActiveTags() { return _activeTags; }

  inline char GetFillChar() const { return _fillChar; }
  inline void SetFillChar(char value) { _fillChar = value; }

  //! Minimal length of filename log segment
  inline size_t GetFillWidthFilename() const { return _fillWidthFilename; }
  inline void SetFillWidthFilename(size_t value) { _fillWidthFilename = value; }

  //! Minimal length of function name log segment
  inline size_t GetFillWidthFunctionName() const { return _fillWidthFunctionName; }
  inline void SetFillWidthFunctionName(size_t value) { _fillWidthFunctionName = value; }

  //! Minimal length of line number log segment
  inline size_t GetFillWidthLineNo() const { return _fillWidthLineNo; }
  inline void SetFillWidthLineNo(size_t value) { _fillWidthLineNo = value; }

  inline bool IsColorized() const { return _isColorized; }
  inline void SetColorized(bool value) { _isColorized = value; }

  inline bool IsLevelWritten() const { return _isLevelWritten; }
  inline void SetLevelWritten(bool value) { _isLevelWritten = value; }

  inline bool IsTagWritten() const { return _isTagWritten; }
  inline void SetTagWritten(bool value) { _isTagWritten = value; }

  inline bool IsSubTagWritten() const { return _isSubTagWritten; }
  inline void SetSubTagWritten(bool value) { _isSubTagWritten = value; }

  inline bool IsFunctionNameWritten() const { return _isFunctionNameWritten; }
  inline void SetFunctionNameWritten(bool value) { _isFunctionNameWritten = value; }

  inline bool IsLocationWritten() const { return _isLocationWritten; }
  inline void SetLocationWritten(bool value) { _isLocationWritten = value; }

  inline bool IsTimestampWritten() const { return _isTimestampWritten; }
  inline void SetTimestampWritten(bool value) { _isTimestampWritten = value; }

  inline bool IsEnabled() const { return _isEnabled; }
  inline void SetEnabled(bool value) { _isEnabled = value; }


  /*------------------------------------------------------------------------------------------------------------------*/
  /*//////// Protected Interface /////////////////////////////////////////////////////////////////////////////////////*/
  /*------------------------------------------------------------------------------------------------------------------*/
private:
  /* Controller */
  /*------------------------------------------------------------------------------------------------------------------*/
  virtual void Write(const std::string& str) {
    if (_ostream) *_ostream << str << std::flush;
  }


  /*------------------------------------------------------------------------------------------------------------------*/
  /*//////// Private Interface ///////////////////////////////////////////////////////////////////////////////////////*/
  /*------------------------------------------------------------------------------------------------------------------*/
private:
  /* Miscellaneous */
  /*------------------------------------------------------------------------------------------------------------------*/
  void WriteColorStart(Level level) {
    switch (level) {
      case Level::ERROR: _ss << PRINT_RED; break;
      case Level::WARNING: _ss << PRINT_YELLOW; break;
      case Level::DEBUG: _ss << PRINT_BLUE; break;
      case Level::TRACE: _ss << PRINT_PURPLE; break;
      case Level::INFO: _ss << PRINT_GREEN; break;
      default: return;
    }
  }
  void WriteColorEnd() { _ss << PRINT_RESET; }

  void WriteLevel(Level level) {
    switch (level) {
      case Level::ERROR: _ss << "[E]"; break;
      case Level::WARNING: _ss << "[W]"; break;
      case Level::DEBUG: _ss << "[D]"; break;
      case Level::TRACE: _ss << "[T]"; break;
      case Level::INFO: _ss << "[I]"; break;
      default: return;
    }
  }
  void WriteTimestamp(const std::chrono::system_clock::time_point& timePoint) {
    // NOLINTBEGIN
    std::time_t nowTime = std::chrono::system_clock::to_time_t(timePoint);

    const auto nowTM = *std::localtime(&nowTime);
    char timestamp[80] {};

    // TODO: Upgrade ASAP
    // http://en.cppreference.com/w/cpp/chrono/c/strftime
    std::strftime(timestamp, sizeof(timestamp), "[%Y.%m.%d %X", &nowTM);

    auto millisecond = uint64_t((double)timePoint.time_since_epoch().count() / (double)std::chrono::system_clock::period::den * 1'000) % 1'000;

    _ss << timestamp << '.' << std::setfill('0') << std::setw(3) << millisecond << ']';
    // NOLINTEND
  }
  void WriteLocation(const std::source_location& location) {
    auto filename = std::filesystem::path(location.file_name()).filename().string();

    _ss << "[" << std::right << std::setfill(_fillChar) << std::setw((int)_fillWidthFilename) << filename << ":" << std::right << std::setfill('0')
        << std::setw((int)_fillWidthLineNo) << location.line() << "]";
  }
  void WriteFunctionName(const std::source_location& location) {
    const std::string fullFunctionName = location.function_name();

    const auto endPosition = fullFunctionName.find('(');
    if (endPosition != std::string::npos) {
      const std::string_view view(fullFunctionName.c_str(), endPosition);
      if (view.find('>') != std::string::npos) {
        static const std::regex REGEX { ".+:([^:>]+(?:<.*>)?)\\([^\\(]+" };

        std::smatch match;
        if (std::regex_search(fullFunctionName, match, REGEX) && match.size() == 2) { _ss << "[" << match[1] << "()]"; }
      } else {
        auto startPosition = view.rfind(':');
        if (startPosition == std::string::npos) startPosition = view.rfind(' ', endPosition);

        if (startPosition != std::string::npos) {
          _ss << "[" << std::setfill('.') << std::setw((int)_fillWidthFunctionName) << fullFunctionName.substr(startPosition + 1, endPosition - startPosition - 1)
              << "()]";
        }
      }
    }
  }


  /* Variables */
  /*------------------------------------------------------------------------------------------------------------------*/
  std::recursive_mutex _recursiveMutex;

  std::ostream* _ostream;
  std::stringstream _ss;

  std::unordered_map<std::string, Level> _activeTags;

  Level _enabledLevels;

  //! Uses std::format()
  // std::string _timestampFormat = "[Y.m.d H:i:s.u]";

  char _fillChar = '.';

  //! Minimal length of filename log segment
  size_t _fillWidthFilename = 20;
  //! Minimal length of function name log segment
  size_t _fillWidthFunctionName = 20;
  //! Minimal length of line number log segment
  size_t _fillWidthLineNo = 4;

  //! Colorization
  bool _isColorized = false;
  //! Log level
  bool _isLevelWritten = true;
  //! Log tag
  bool _isTagWritten = true;
  //! Log sub tag
  bool _isSubTagWritten = true;
  //! Log function name
  bool _isFunctionNameWritten = true;
  //! Log filename and line number
  bool _isLocationWritten = true;
  //! Log with timestamp
  bool _isTimestampWritten = true;

  bool _isEnabled = true;
};
}