// Copyright Open Logistics Foundation
//
// Licensed under the Open Logistics Foundation License 1.3.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: OLFL-1.3
//
// This file contains the BaseQueryHandler class.
//
//

#ifndef PUBLIC_VDA5050_2B_2B_HANDLER_BASE_QUERY_HANDLER_H_
#define PUBLIC_VDA5050_2B_2B_HANDLER_BASE_QUERY_HANDLER_H_
#include <vda5050/Error.h>
#include <vda5050/Node.h>

#include <list>
#include <mutex>
#include <optional>

namespace vda5050pp::handler {

///
///\brief The BaseQueryHandler class can be overwritten to let the library automatically handle
/// QueryEvents. Note, that you can configure a default behaviour for query answering.
/// (QueryEventHandlerSubConfig)
///
/// NOTE: The library will call the member functions of this class with one dedicated thread.
/// The user has to make sure to return from the functions in order to receive the next function
/// call.
///
class BaseQueryHandler {
public:
  ///
  ///\brief The result type for QueryPauseable and QueryResumable events.
  ///
  struct QueryPauseResumeResult {
    ///
    ///\brief A list of errors (empty iff. pause/resume is allowed)
    ///
    std::list<vda5050::Error> errors;
    ///
    ///\brief Notify via actions via ActionPause or ActionResume events.
    ///
    bool notify = true;
  };

private:
  std::mutex call_query_pauseable_mutex_;
  std::mutex call_query_resumable_mutex_;
  std::mutex call_query_accept_zone_set_mutex_;
  std::mutex call_query_node_trivially_reachable_mutex_;
  bool overwrite_query_pauseable_ = false;
  bool overwrite_query_resumable_ = false;
  bool overwrite_query_accept_zone_set_ = false;
  bool overwrite_query_node_trivially_reachable_ = false;

public:
  virtual ~BaseQueryHandler() = default;

  ///
  ///\brief A call wrapper around queryPauseable() to detect user code.
  ///
  ///\return std::optional<QueryPauseResumeResult>
  ///
  std::optional<QueryPauseResumeResult> callQueryPauseable();

  ///
  ///\brief A call wrapper around queryResumable() to detect user code.
  ///
  ///\return std::optional<QueryPauseResumeResult>
  ///
  std::optional<QueryPauseResumeResult> callQueryResumable();

  ///
  ///\brief A call wrapper around queryAcceptZoneSet() to detect user code.
  ///
  ///\param zone_set_id the zoneSetId to check.
  ///\return std::optional<std::list<vda5050::Error>>
  ///
  std::optional<std::list<vda5050::Error>> callQueryAcceptZoneSet(std::string_view zone_set_id);

  ///
  ///\brief A call wrapper around queryNodeTriviallyReachable() to detect user code.
  ///
  ///\param node the node to check.
  ///\return std::optional<std::list<vda5050::Error>>
  ///
  std::optional<std::list<vda5050::Error>> callQueryNodeTriviallyReachable(
      std::shared_ptr<const vda5050::Node> node);

  ///
  ///\brief This function may be overwritten to handle a QueryPauseableEvent.
  ///
  ///\return QueryPauseResumeResult the result
  ///
  virtual QueryPauseResumeResult queryPauseable();

  ///
  ///\brief This function may be overwritten to handle a QueryResumableEvent.
  ///
  ///\return QueryPauseResumeResult the result
  ///
  virtual QueryPauseResumeResult queryResumable();

  ///
  ///\brief This file can be overwritten to handle a QueryAcceptZoneSetEvent.
  ///
  ///\param zone_set_id the order.zoneSetId of the currently validated order.
  ///\return std::list<vda5050::Error> empty iff. the zoneSetId can be accepted.
  ///
  virtual std::list<vda5050::Error> queryAcceptZoneSet(std::string_view zone_set_id);

  ///
  ///\brief The function can be overwritten to handle a QueryNodeTriviallyReachableEvent.
  /// Trivially reachable means, that the AGV considers itself to be on the node or
  /// is in the deviation range of the node. The agv needs to be able to navigate to adjacent nodes
  /// without information about this node. (See VDA5050 6.6.1)
  ///
  ///\param node the node which is in need to be checked if it is trivially reachable.
  ///\return std::list<vda5050::Error> empty iff. the node is trivially reachable.
  ///
  virtual std::list<vda5050::Error> queryNodeTriviallyReachable(
      std::shared_ptr<const vda5050::Node> node);
};

}  // namespace vda5050pp::handler

#endif  // PUBLIC_VDA5050_2B_2B_HANDLER_BASE_QUERY_HANDLER_H_
