#pragma once
#include <any>
#include <functional>
#include <vector>

namespace FunctionTrait {
namespace details {

template <int> struct Index {};

template <int, typename... Ts> struct TypesImpl;

template <int i, typename T, typename... Ts>
struct TypesImpl<i, T, Ts...> : TypesImpl<i + 1, Ts...> {
protected:
  static T get(Index<i>);
  using TypesImpl<i + 1, Ts...>::get;
};

template <int i, typename T> struct TypesImpl<i, T> {
protected:
  static T get(Index<i>);

public:
  constexpr static int count = i + 1;
};

template <typename T> struct TypeContainer {
  using Type = T;
};

} // namespace details

template <typename... Ts> struct Types : details::TypesImpl<0, Ts...> {
  template <int i> struct Container {
    static_assert(i < details::TypesImpl<0, Ts...>::count, "Invalid index");
    using Type =
        decltype(details::TypesImpl<0, Ts...>::get(details::Index<i>{}));
  };
};

template <> struct Types<> {
  constexpr static int count = 0;
  template <int> struct Container {
    static_assert(false, "Invalid index");
  };
};

template <typename T> struct FunctionTrait;

template <typename R, typename... A> struct FunctionTrait<R (*)(A...)> {
  using RetType = R;
  using ArgTypes = Types<A...>;
  constexpr static bool IsFunctor = false;
  constexpr static bool IsConst = false;
  constexpr static bool IsVolatile = false;
  constexpr static bool IsConstVolatile = IsConst && IsVolatile;
};

template <typename R, typename T, typename... A>
struct FunctionTrait<R (T::*)(A...)> {
  using RetType = R;
  using ArgTypes = Types<A...>;
  using ClassType = T;
  constexpr static bool IsFunctor = true;
  constexpr static bool IsConst = false;
  constexpr static bool IsVolatile = false;
  constexpr static bool IsConstVolatile = IsConst && IsVolatile;
};

template <typename R, typename T, typename... A>
struct FunctionTrait<R (T::*)(A...) const> {
  using RetType = R;
  using ArgTypes = Types<A...>;
  using ClassType = T;
  constexpr static bool IsFunctor = true;
  constexpr static bool IsConst = true;
  constexpr static bool IsVolatile = false;
  constexpr static bool IsConstVolatile = IsConst && IsVolatile;
};

template <typename R, typename T, typename... A>
struct FunctionTrait<R (T::*)(A...) volatile> {
  using RetType = R;
  using ArgTypes = Types<A...>;
  using ClassType = T;
  constexpr static bool IsFunctor = true;
  constexpr static bool IsConst = false;
  constexpr static bool IsVolatile = true;
  constexpr static bool IsConstVolatile = IsConst && IsVolatile;
};

template <typename R, typename T, typename... A>
struct FunctionTrait<R (T::*)(A...) const volatile> {
  using RetType = R;
  using ArgTypes = Types<A...>;
  using ClassType = T;
  constexpr static bool IsFunctor = true;
  constexpr static bool IsConst = true;
  constexpr static bool IsVolatile = true;
  constexpr static bool IsConstVolatile = IsConst && IsVolatile;
};

template <typename T>
struct FunctionTrait<std::function<T>>
    : FunctionTrait<decltype(&std::declval<T>())> {};
} // namespace FunctionTrait

namespace AnyFunction {
using AnyFunction = std::function<std::any(const std::vector<std::any> &)>;

namespace details {
constexpr auto call_any(auto f, const std::vector<std::any> &args,
                        auto &&...temps) {
  if constexpr (sizeof...(temps) ==
                FunctionTrait::FunctionTrait<
                    std::remove_cvref_t<decltype(f)>>::ArgTypes::count) {
    if constexpr (sizeof...(temps) == 0) {
      if constexpr (std::is_same_v<
                        typename FunctionTrait::FunctionTrait<
                            std::remove_cvref_t<decltype(f)>>::RetType,
                        void>) {
        f();
        return std::any{};
      } else {
        return std::make_any<typename FunctionTrait::FunctionTrait<
            std::remove_cvref_t<decltype(f)>>::RetType>(std::move(f()));
      }
    } else {
      if constexpr (std::is_same_v<
                        typename FunctionTrait::FunctionTrait<
                            std::remove_cvref_t<decltype(f)>>::RetType,
                        void>) {
        f((*temps)...);
        return std::any{};
      } else {
        return std::make_any<typename FunctionTrait::FunctionTrait<
            std::remove_cvref_t<decltype(f)>>::RetType>(
            std::move(f((*temps)...)));
      }
    }
  } else {
    if constexpr (sizeof...(temps) == 0) {
      const auto &t = std::any_cast<
          typename FunctionTrait::FunctionTrait<std::remove_cvref_t<
              decltype(f)>>::ArgTypes::template Container<0>::Type>(args[0]);
      return call_any(f, args, &t);
    } else {
      const auto &t =
          std::any_cast<typename FunctionTrait::FunctionTrait<
              std::remove_cvref_t<decltype(f)>>::ArgTypes::
                            template Container<sizeof...(temps)>::Type>(
              args[sizeof...(temps)]);
      return call_any(f, args, std::forward<decltype(temps)>(temps)..., &t);
    }
  }
}

} // namespace details
constexpr AnyFunction make_any_function(auto &&f) {
  std::function fn{std::forward<decltype(f)>(f)};
  return [fn](const std::vector<std::any> &args) -> std::any {
    return details::call_any(fn, args);
  };
}
} // namespace AnyFunction
