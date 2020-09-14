###############################################################################
# Headers
###############################################################################

file(GLOB HEADERS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.h)

install(FILES ${HEADERS} DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/nanomsg)

###############################################################################
# Libraries
###############################################################################

set (NN_SOURCES
    core/ep.c
    core/epbase.c
    core/global.c
    core/pipe.c
    core/poll.c
    core/sock.c
    core/sockbase.c
    core/device.c
    core/symbol.c

    aio/ctx.c
    aio/fsm.c
    aio/poller.c
    aio/poller_epoll.inc
    aio/poller_kqueue.inc
    aio/poller_poll.inc
    aio/pool.c
    aio/timer.c
    aio/timerset.c
    aio/usock.c
    aio/usock_posix.inc
    aio/usock_win.inc
    aio/worker.c
    aio/worker_posix.inc
    aio/worker_win.inc

    utils/alloc.c
    utils/atomic.c
    utils/chunk.c
    utils/chunkref.c
    utils/clock.c
    utils/closefd.c
    utils/efd.c
    utils/efd_eventfd.inc
    utils/efd_pipe.inc
    utils/efd_socketpair.inc
    utils/efd_win.inc
    utils/err.c
    utils/glock.c
    utils/hash.c
    utils/list.c
    utils/msg.c
    utils/mutex.c
    utils/queue.c
    utils/random.c
    utils/sem.c
    utils/sleep.c
    utils/thread.c
    utils/thread_posix.inc
    utils/thread_win.inc
    utils/wire.c

    protocols/utils/dist.c
    protocols/utils/excl.c
    protocols/utils/fq.c
    protocols/utils/lb.c
    protocols/utils/priolist.c

    protocols/bus/bus.c
    protocols/bus/xbus.c

    protocols/pipeline/push.c
    protocols/pipeline/pull.c
    protocols/pipeline/xpull.c
    protocols/pipeline/xpush.c

    protocols/pair/pair.c
    protocols/pair/xpair.c

    protocols/pubsub/pub.c
    protocols/pubsub/sub.c
    protocols/pubsub/trie.c
    protocols/pubsub/xpub.c
    protocols/pubsub/xsub.c

    protocols/reqrep/req.c
    protocols/reqrep/rep.c
    protocols/reqrep/xrep.c
    protocols/reqrep/xreq.c

    protocols/survey/respondent.c
    protocols/survey/surveyor.c
    protocols/survey/xrespondent.c
    protocols/survey/xsurveyor.c

    transports/utils/backoff.c
    transports/utils/dns.c
    transports/utils/dns_getaddrinfo.inc
    transports/utils/dns_getaddrinfo_a.inc
    transports/utils/iface.c
    transports/utils/literal.c
    transports/utils/port.c
    transports/utils/streamhdr.c

    transports/inproc/binproc.c
    transports/inproc/cinproc.c
    transports/inproc/inproc.c
    transports/inproc/ins.c
    transports/inproc/msgqueue.c
    transports/inproc/sinproc.c

    transports/ipc/aipc.c
    transports/ipc/bipc.c
    transports/ipc/cipc.c
    transports/ipc/ipc.c
    transports/ipc/sipc.c

    transports/tcp/atcp.c
    transports/tcp/btcp.c
    transports/tcp/ctcp.c
    transports/tcp/stcp.c
    transports/tcp/tcp.c
)

add_library (nanomsg ${NN_SOURCES})

target_link_libraries(nanomsg anl ${CMAKE_THREAD_LIBS_INIT})

###############################################################################
# Install
###############################################################################

install(TARGETS nanomsg
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin)

###############################################################################
# Directories
###############################################################################

add_subdirectory(aio)
add_subdirectory(core)
add_subdirectory(protocols)
add_subdirectory(transports)
add_subdirectory(utils)
