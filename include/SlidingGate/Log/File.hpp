#pragma once
#include <SlidingGate/Log/Stream.hpp>

namespace SlidingGate{
class Log::File : public Log::Stream {
  /*------------------------------------------------------------------------------------------------------------------*/
  /*//////// Public Interface ////////////////////////////////////////////////////////////////////////////////////////*/
  /*------------------------------------------------------------------------------------------------------------------*/
public:
  /* Constructors / Destructor */
  /*------------------------------------------------------------------------------------------------------------------*/
  File(std::filesystem::path filePath, Level levels = Level::ALL, std::string description = "", std::ios_base::openmode openMode = std::ios_base::out)
    : Stream(nullptr, levels)
    , _filePath(std::move(filePath))
    , _description(std::move(description))
    , _openMode(openMode) {}
  File(const File&) = delete;
  File(File&&) = delete;


  /* Operators */
  /*------------------------------------------------------------------------------------------------------------------*/
  File& operator=(const File&) = delete;
  File& operator=(File&&) = delete;


  /* Controller */
  /*------------------------------------------------------------------------------------------------------------------*/
  bool Open() {
    if (_ofstream) return true;

    // Open File

    _ofstream = std::make_shared<std::ofstream>();
    _ofstream->rdbuf()->pubsetbuf(nullptr, 0); // G++ NEEDS this order!!!
    _ofstream->open(_filePath, _openMode | std::ios::binary);
    // *_ofstream << std::unitbuf;

    if (!IsOpen()) {
      HandleError();
      return false;
    }

    std::filesystem::permissions(_filePath,
      std::filesystem::perms::owner_read | std::filesystem::perms::owner_write | std::filesystem::perms::group_read | std::filesystem::perms::group_write
        | std::filesystem::perms::others_read | std::filesystem::perms::others_write,
      std::filesystem::perm_options::replace);

    // Read current file byte count

    _currentByteCount = _ofstream->tellp();
    if (_currentByteCount < 0) {
      HandleError();
      return false;
    }

    // Write Header

    if (!_header.empty() && !_ofstream->write(_header.c_str(), (std::streamsize)_header.size())) {
      HandleError();
      return false;
    }
    _currentByteCount += _header.size();

    // Write Record Count

    _completeRecordCount += _currentRecordCount;
    _currentRecordCount = 0;

    if (_completeRecordCount > 0) {
      std::array<char, 512> record {};

      auto byteCount = snprintf(record.data(), record.size(), "%zd records already written\n\n", _completeRecordCount);
      if (!_ofstream->write(record.data(), byteCount)) {
        HandleError();
        return false;
      }
      _currentByteCount += byteCount;
    }

    return true;
  }
  void Close() {
    _currentByteCount = 0;

    if (_ofstream) _ofstream->close();
    _ofstream = nullptr;
  }


  /* Getter / Setter */
  /*------------------------------------------------------------------------------------------------------------------*/
  inline const std::filesystem::path& GetFilePath() const { return _filePath; }

  [[nodiscard]] inline const std::string& GetDescription() const { return _description; }
  inline void SetDescription(const std::string& value) { _description = value; }

  inline const std::string& GetHeader() const { return _header; }
  inline void SetHeader(const std::string& value) { _header = value; }

  inline size_t GetMaxByteCount() const { return _maxByteCount; }
  inline void SetMaxByteCount(size_t value) { _maxByteCount = value; }

  [[nodiscard]] inline bool IsError() const { return (_isError || (_ofstream && _ofstream->bad())); }
  [[nodiscard]] inline bool IsOpen() const { return (_ofstream && _ofstream->is_open()); }


  /*------------------------------------------------------------------------------------------------------------------*/
  /*//////// Private Interface ///////////////////////////////////////////////////////////////////////////////////////*/
  /*------------------------------------------------------------------------------------------------------------------*/
private:
  /* Log::Stream */
  /*------------------------------------------------------------------------------------------------------------------*/
  void Write(const std::string& str) override {
    if (!_ofstream && (_isError || !Open())) return;

    if (!_ofstream->write(str.c_str(), (std::streamsize)str.size())) return HandleError();
    *_ofstream << std::flush;

    ++_currentRecordCount;

    _currentByteCount += str.size();
    if (_currentByteCount < _maxByteCount) return;

    // File too big. Rename old and create a new file.

    Close();

    const auto renamedFileName = _filePath.string() + ".1";

    std::error_code ec;
    std::filesystem::remove(renamedFileName, ec);
    if (ec) return HandleError();

    std::filesystem::rename(_filePath, renamedFileName, ec);
    if (ec) return HandleError();

    Open();
  }


  /* Miscellaneous */
  /*------------------------------------------------------------------------------------------------------------------*/
  void HandleError() {
    Close();
    _isError = true;
  }


  /* Variables */
  /*------------------------------------------------------------------------------------------------------------------*/
  std::shared_ptr<std::ofstream> _ofstream;

  std::filesystem::path _filePath;
  std::ios_base::openmode _openMode;

  std::string _description;
  std::string _header;

  size_t _completeRecordCount = 0;
  size_t _currentRecordCount = 0;

  size_t _currentByteCount = 0;
  size_t _maxByteCount = -1;

  bool _isError = false;
};
}