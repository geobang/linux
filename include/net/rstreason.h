/* SPDX-License-Identifier: GPL-2.0-or-later */

#ifndef _LINUX_RSTREASON_H
#define _LINUX_RSTREASON_H
#include <net/dropreason-core.h>
#include <uapi/linux/mptcp.h>

#define DEFINE_RST_REASON(FN, FNe)	\
	FN(NOT_SPECIFIED)		\
	FN(NO_SOCKET)			\
	FN(TCP_INVALID_ACK_SEQUENCE)	\
	FN(TCP_RFC7323_PAWS)		\
	FN(TCP_TOO_OLD_ACK)		\
	FN(TCP_ACK_UNSENT_DATA)		\
	FN(TCP_FLAGS)			\
	FN(TCP_OLD_ACK)			\
	FN(TCP_ABORT_ON_DATA)		\
	FN(TCP_TIMEWAIT_SOCKET)		\
	FN(INVALID_SYN)			\
	FN(MPTCP_RST_EUNSPEC)		\
	FN(MPTCP_RST_EMPTCP)		\
	FN(MPTCP_RST_ERESOURCE)		\
	FN(MPTCP_RST_EPROHIBIT)		\
	FN(MPTCP_RST_EWQ2BIG)		\
	FN(MPTCP_RST_EBADPERF)		\
	FN(MPTCP_RST_EMIDDLEBOX)	\
	FN(ERROR)			\
	FNe(MAX)

/**
 * enum sk_rst_reason - the reasons of socket reset
 *
 * The reasons of sk reset, which are used in DCCP/TCP/MPTCP protocols.
 *
 * There are three parts in order:
 * 1) skb drop reasons: relying on drop reasons for such as passive reset
 * 2) independent reset reasons: such as active reset reasons
 * 3) reset reasons in MPTCP: only for MPTCP use
 */
enum sk_rst_reason {
	/* Refer to include/net/dropreason-core.h
	 * Rely on skb drop reasons because it indicates exactly why RST
	 * could happen.
	 */
	/** @SK_RST_REASON_NOT_SPECIFIED: reset reason is not specified */
	SK_RST_REASON_NOT_SPECIFIED,
	/** @SK_RST_REASON_NO_SOCKET: no valid socket that can be used */
	SK_RST_REASON_NO_SOCKET,
	/**
	 * @SK_RST_REASON_TCP_INVALID_ACK_SEQUENCE: Not acceptable ACK SEQ
	 * field because ack sequence is not in the window between snd_una
	 * and snd_nxt
	 */
	SK_RST_REASON_TCP_INVALID_ACK_SEQUENCE,
	/**
	 * @SK_RST_REASON_TCP_RFC7323_PAWS: PAWS check, corresponding to
	 * LINUX_MIB_PAWSESTABREJECTED, LINUX_MIB_PAWSACTIVEREJECTED
	 */
	SK_RST_REASON_TCP_RFC7323_PAWS,
	/** @SK_RST_REASON_TCP_TOO_OLD_ACK: TCP ACK is too old */
	SK_RST_REASON_TCP_TOO_OLD_ACK,
	/**
	 * @SK_RST_REASON_TCP_ACK_UNSENT_DATA: TCP ACK for data we haven't
	 * sent yet
	 */
	SK_RST_REASON_TCP_ACK_UNSENT_DATA,
	/** @SK_RST_REASON_TCP_FLAGS: TCP flags invalid */
	SK_RST_REASON_TCP_FLAGS,
	/** @SK_RST_REASON_TCP_OLD_ACK: TCP ACK is old, but in window */
	SK_RST_REASON_TCP_OLD_ACK,
	/**
	 * @SK_RST_REASON_TCP_ABORT_ON_DATA: abort on data
	 * corresponding to LINUX_MIB_TCPABORTONDATA
	 */
	SK_RST_REASON_TCP_ABORT_ON_DATA,

	/* Here start with the independent reasons */
	/** @SK_RST_REASON_TCP_TIMEWAIT_SOCKET: happen on the timewait socket */
	SK_RST_REASON_TCP_TIMEWAIT_SOCKET,
	/**
	 * @SK_RST_REASON_INVALID_SYN: receive bad syn packet
	 * RFC 793 says if the state is not CLOSED/LISTEN/SYN-SENT then
	 * "fourth, check the SYN bit,...If the SYN is in the window it is
	 * an error, send a reset"
	 */
	SK_RST_REASON_INVALID_SYN,

	/* Copy from include/uapi/linux/mptcp.h.
	 * These reset fields will not be changed since they adhere to
	 * RFC 8684. So do not touch them. I'm going to list each definition
	 * of them respectively.
	 */
	/**
	 * @SK_RST_REASON_MPTCP_RST_EUNSPEC: Unspecified error.
	 * This is the default error; it implies that the subflow is no
	 * longer available. The presence of this option shows that the
	 * RST was generated by an MPTCP-aware device.
	 */
	SK_RST_REASON_MPTCP_RST_EUNSPEC,
	/**
	 * @SK_RST_REASON_MPTCP_RST_EMPTCP: MPTCP-specific error.
	 * An error has been detected in the processing of MPTCP options.
	 * This is the usual reason code to return in the cases where a RST
	 * is being sent to close a subflow because of an invalid response.
	 */
	SK_RST_REASON_MPTCP_RST_EMPTCP,
	/**
	 * @SK_RST_REASON_MPTCP_RST_ERESOURCE: Lack of resources.
	 * This code indicates that the sending host does not have enough
	 * resources to support the terminated subflow.
	 */
	SK_RST_REASON_MPTCP_RST_ERESOURCE,
	/**
	 * @SK_RST_REASON_MPTCP_RST_EPROHIBIT: Administratively prohibited.
	 * This code indicates that the requested subflow is prohibited by
	 * the policies of the sending host.
	 */
	SK_RST_REASON_MPTCP_RST_EPROHIBIT,
	/**
	 * @SK_RST_REASON_MPTCP_RST_EWQ2BIG: Too much outstanding data.
	 * This code indicates that there is an excessive amount of data
	 * that needs to be transmitted over the terminated subflow while
	 * having already been acknowledged over one or more other subflows.
	 * This may occur if a path has been unavailable for a short period
	 * and it is more efficient to reset and start again than it is to
	 * retransmit the queued data.
	 */
	SK_RST_REASON_MPTCP_RST_EWQ2BIG,
	/**
	 * @SK_RST_REASON_MPTCP_RST_EBADPERF: Unacceptable performance.
	 * This code indicates that the performance of this subflow was
	 * too low compared to the other subflows of this Multipath TCP
	 * connection.
	 */
	SK_RST_REASON_MPTCP_RST_EBADPERF,
	/**
	 * @SK_RST_REASON_MPTCP_RST_EMIDDLEBOX: Middlebox interference.
	 * Middlebox interference has been detected over this subflow,
	 * making MPTCP signaling invalid. For example, this may be sent
	 * if the checksum does not validate.
	 */
	SK_RST_REASON_MPTCP_RST_EMIDDLEBOX,

	/** @SK_RST_REASON_ERROR: unexpected error happens */
	SK_RST_REASON_ERROR,

	/**
	 * @SK_RST_REASON_MAX: Maximum of socket reset reasons.
	 * It shouldn't be used as a real 'reason'.
	 */
	SK_RST_REASON_MAX,
};

/* Convert skb drop reasons to enum sk_rst_reason type */
static inline enum sk_rst_reason
sk_rst_convert_drop_reason(enum skb_drop_reason reason)
{
	switch (reason) {
	case SKB_DROP_REASON_NOT_SPECIFIED:
		return SK_RST_REASON_NOT_SPECIFIED;
	case SKB_DROP_REASON_NO_SOCKET:
		return SK_RST_REASON_NO_SOCKET;
	case SKB_DROP_REASON_TCP_INVALID_ACK_SEQUENCE:
		return SK_RST_REASON_TCP_INVALID_ACK_SEQUENCE;
	case SKB_DROP_REASON_TCP_RFC7323_PAWS:
		return SK_RST_REASON_TCP_RFC7323_PAWS;
	case SKB_DROP_REASON_TCP_TOO_OLD_ACK:
		return SK_RST_REASON_TCP_TOO_OLD_ACK;
	case SKB_DROP_REASON_TCP_ACK_UNSENT_DATA:
		return SK_RST_REASON_TCP_ACK_UNSENT_DATA;
	case SKB_DROP_REASON_TCP_FLAGS:
		return SK_RST_REASON_TCP_FLAGS;
	case SKB_DROP_REASON_TCP_OLD_ACK:
		return SK_RST_REASON_TCP_OLD_ACK;
	case SKB_DROP_REASON_TCP_ABORT_ON_DATA:
		return SK_RST_REASON_TCP_ABORT_ON_DATA;
	default:
		/* If we don't have our own corresponding reason */
		return SK_RST_REASON_NOT_SPECIFIED;
	}
}
#endif
