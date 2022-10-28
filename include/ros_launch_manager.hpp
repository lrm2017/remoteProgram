#ifndef ROSLAUNCHMANGER_H
#define ROSLAUNCHMANGER_H

#endif // ROSLAUNCHMANGER_H

#include <unistd.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/wait.h>

#include <cstdint>
#include <memory>
#include <thread>
#include <mutex>
#include <vector>
#include <algorithm>
#include <numeric>

#include "ros/ros.h"
#include "ros/console.h"
#include "rospack/rospack.h"

// 将vector转换为命令
namespace util {
    template <typename ReturnType, typename... Args>
    struct function_traits_defs {
        static constexpr size_t arity = sizeof...(Args);

        using result_type = ReturnType;

        template <size_t i>
        struct arg {
            using type = typename std::tuple_element<i, std::tuple<Args...>>::type;
        };
    };

    template <typename T>
    struct function_traits_impl;

    template <typename ReturnType, typename... Args>
    struct function_traits_impl<ReturnType(Args...)>
            : function_traits_defs<ReturnType, Args...> {};

    template <typename ReturnType, typename... Args>
    struct function_traits_impl<ReturnType(*)(Args...)>
            : function_traits_defs<ReturnType, Args...> {};

    template <typename ClassType, typename ReturnType, typename... Args>
    struct function_traits_impl<ReturnType(ClassType::*)(Args...)>
            : function_traits_defs<ReturnType, Args...> {};

    template <typename ClassType, typename ReturnType, typename... Args>
    struct function_traits_impl<ReturnType(ClassType::*)(Args...) const>
            : function_traits_defs<ReturnType, Args...> {};

    template <typename ClassType, typename ReturnType, typename... Args>
    struct function_traits_impl<ReturnType(ClassType::*)(Args...) const&>
            : function_traits_defs<ReturnType, Args...> {};

    template <typename ClassType, typename ReturnType, typename... Args>
    struct function_traits_impl<ReturnType(ClassType::*)(Args...) const&&>
            : function_traits_defs<ReturnType, Args...> {};

    template <typename ClassType, typename ReturnType, typename... Args>
    struct function_traits_impl<ReturnType(ClassType::*)(Args...) volatile>
            : function_traits_defs<ReturnType, Args...> {};

    template <typename ClassType, typename ReturnType, typename... Args>
    struct function_traits_impl<ReturnType(ClassType::*)(Args...) volatile&>
            : function_traits_defs<ReturnType, Args...> {};

    template <typename ClassType, typename ReturnType, typename... Args>
    struct function_traits_impl<ReturnType(ClassType::*)(Args...) volatile&&>
            : function_traits_defs<ReturnType, Args...> {};

    template <typename ClassType, typename ReturnType, typename... Args>
    struct function_traits_impl<ReturnType(ClassType::*)(Args...) const volatile>
            : function_traits_defs<ReturnType, Args...> {};

    template <typename ClassType, typename ReturnType, typename... Args>
    struct function_traits_impl<ReturnType(ClassType::*)(Args...) const volatile&>
            : function_traits_defs<ReturnType, Args...> {};

    template <typename ClassType, typename ReturnType, typename... Args>
    struct function_traits_impl<ReturnType(ClassType::*)(Args...) const volatile&&>
            : function_traits_defs<ReturnType, Args...> {};

//    struct function_traits_impl<

    template <typename T, typename V = void>
    struct function_traits
            : function_traits_impl<T> {};

    template <typename T>
    struct function_traits<T, decltype((void)&T::operator())>
            : function_traits_impl<decltype(&T::operator())> {};

    template <size_t... Indices>
    struct indices {
        using next = indices<Indices..., sizeof...(Indices)>;
    };
    template <size_t N>
    struct build_indices {
        using type = typename build_indices<N - 1>::type::next;
    };
    template <>
    struct build_indices<0> {
        using type = indices<>;
    };
    template <size_t N>
    using BuildIndices = typename build_indices<N>::type;

    namespace details {
        template <typename FuncType,
                typename VecType,
                size_t... I,
                typename Traits = function_traits<FuncType>,
                typename ReturnT = typename Traits::result_type>
        ReturnT do_call(FuncType& func, const VecType& args, indices<I...>) {
            return func(args[I]...);
        }
    }  // namespace details

    template <typename FuncType, typename VecType, typename Traits = function_traits<FuncType>>
    decltype(auto) unpack_caller(FuncType& func, const VecType& args) {
        return details::do_call(func, args, BuildIndices<Traits::arity>());
    }
}  // namespace util



class ROSLaunchManager
{
    std::vector<pid_t> m_pids;

    std::atomic<bool> m_running;
    std::thread m_thread;
    std::mutex m_mutex;

public:
    ROSLaunchManager(ROSLaunchManager const &)
    {
    }

    ROSLaunchManager()
    {
        std::atomic_init(&m_running, true);

        m_thread = std::thread(&ROSLaunchManager::wait, this);
    }

    ~ROSLaunchManager()
    {
        if (m_running.load()) {
            m_running.store(false);

            if (m_thread.joinable()) {
                m_thread.join();
            }
        }
    }

//    template<typename... Args>
    pid_t start(std::vector<char*> args_vector)
    {
//        args_vector.insert(args_vector.end(), NULL);
        std::vector<std::string> vec(args_vector.data(),args_vector.data()+args_vector.size());
//        vec.insert(vec.begin(),1,ros);
        if (args_vector.size() > 0) {
            pid_t pid = ::fork();

            if (pid == 0) { // 子进程
                ::setsid();

                ::signal(SIGINT, SIG_IGN);

                ::fclose(stdout);
                ::fclose(stdin);
                ::fclose(stderr);
                int state = ::execvp(args_vector[0], &args_vector[0]);
                std::cout << "state:" << state << std::endl;
//                util::unpack_caller(::execlp, vec);   //
//                ::execlp(ros.c_str(), ros.c_str(), args..., nullptr);   // 在环境下调用ros文件，执行ros文件
            }
            else {
                std::scoped_lock<std::mutex> scoped_lock(m_mutex);
                // 
                std::string args_string = std::accumulate(std::next(std::begin(vec)), std::end(vec), vec[0], [](std::string lhs, std::string rhs) -> std::string { return lhs + " " + rhs; });

                ROS_INFO("Starting \"%s\" with PID %d", args_string.c_str(), pid);

                m_pids.push_back(pid);
            }

            return pid;
        }
        else {
            throw std::runtime_error("ROSLaunchManager::start - No arguments provided");
        }
    }

    void stop(pid_t const &pid, int32_t const &signal)
    {
        std::scoped_lock<std::mutex> scoped_lock(m_mutex);

        auto pid_it = std::find(std::begin(m_pids), std::end(m_pids), pid);

        if (pid_it != m_pids.end()) {
            ::kill(pid, signal);

            ROS_INFO("Stopping process with PID %d and signal %d", pid, signal);
        }
        else {
            throw std::runtime_error("ROSLaunchManager::stop - PID " + std::to_string(pid) + " not found");
        }
    }

private:
    void wait()
    {
        while (m_running.load()) {
            std::scoped_lock<std::mutex> scoped_lock(m_mutex);

            for (auto pid_it = std::begin(m_pids); pid_it != std::end(m_pids); ++pid_it) {
                pid_t const pid = *pid_it;

                int32_t status;

                if (::waitpid(pid, &status, WUNTRACED | WCONTINUED | WNOHANG) == pid) {
                    if (WIFEXITED(status)) {
                        ROS_INFO("PID %d exited with status %d", pid, WEXITSTATUS(status));

                        pid_it = m_pids.erase(pid_it);

                        if (pid_it == std::end(m_pids)) {
                            break;
                        }
                    }
                    else if (WIFSIGNALED(status)) {
                        ROS_INFO("PID %d killed with signal %d", pid, WTERMSIG(status));

                        pid_it = m_pids.erase(pid_it);

                        if (pid_it == std::end(m_pids)) {
                            break;
                        }
                    }
                    else if (WIFSTOPPED(status)) {
                        ROS_INFO("PID %d stopped with signal %d", pid, WSTOPSIG(status));
                    }
                    else if (WIFCONTINUED(status)) {
                        ROS_INFO("PID %d continued"   , pid);
                    }
                }
            }
        }

        std::scoped_lock<std::mutex> scoped_lock(m_mutex);

        for (pid_t const &pid : m_pids) {
            ::kill(pid, SIGINT);

            int32_t status;

            ::waitpid(pid, &status, 0);
        }
    }
};

