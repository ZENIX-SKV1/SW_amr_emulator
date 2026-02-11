
#ifndef PAHO_MQTTPP_EXPORT_H
#define PAHO_MQTTPP_EXPORT_H

#ifdef PAHO_MQTTPP_STATIC_DEFINE
#  define PAHO_MQTTPP_EXPORT
#  define PAHO_MQTTPP_NO_EXPORT
#else
#  ifndef PAHO_MQTTPP_EXPORT
#    ifdef paho_mqttpp3_shared_EXPORTS
        /* We are building this library */
#      define PAHO_MQTTPP_EXPORT __attribute__((visibility("default")))
#    else
        /* We are using this library */
#      define PAHO_MQTTPP_EXPORT __attribute__((visibility("default")))
#    endif
#  endif

#  ifndef PAHO_MQTTPP_NO_EXPORT
#    define PAHO_MQTTPP_NO_EXPORT __attribute__((visibility("hidden")))
#  endif
#endif

#ifndef PAHO_MQTTPP_DEPRECATED
#  define PAHO_MQTTPP_DEPRECATED __attribute__ ((__deprecated__))
#endif

#ifndef PAHO_MQTTPP_DEPRECATED_EXPORT
#  define PAHO_MQTTPP_DEPRECATED_EXPORT PAHO_MQTTPP_EXPORT PAHO_MQTTPP_DEPRECATED
#endif

#ifndef PAHO_MQTTPP_DEPRECATED_NO_EXPORT
#  define PAHO_MQTTPP_DEPRECATED_NO_EXPORT PAHO_MQTTPP_NO_EXPORT PAHO_MQTTPP_DEPRECATED
#endif

#if 0 /* DEFINE_NO_DEPRECATED */
#  ifndef PAHO_MQTTPP_NO_DEPRECATED
#    define PAHO_MQTTPP_NO_DEPRECATED
#  endif
#endif

#endif /* PAHO_MQTTPP_EXPORT_H */
