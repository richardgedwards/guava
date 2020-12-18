#pragma once

#ifdef __cpp_exceptions

#include "esp_err.h"
#include <stdexcept>

namespace idf {

#define THROW(error_) { esp_err_t result = error_; if (result != ESP_OK) throw idf::ESPException(result); }


struct ESPException : public std::exception {
    ESPException(esp_err_t error) : std::exception(), _error(error) {}
    esp_err_t _error;
};

}

#endif // __cpp_exceptions
