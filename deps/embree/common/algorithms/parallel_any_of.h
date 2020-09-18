// Copyright 2009-2020 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <functional>
#include "parallel_reduce.h"

namespace embree
{

    template<typename Index, class UnaryPredicate>
    __forceinline bool parallel_any_of (Index first, Index last, UnaryPredicate pred)
    {
        bool ret = false;

#if defined(TASKING_TBB)
        tbb::parallel_for(tbb::blocked_range<size_t>{first, last}, [&ret,pred](const tbb::blocked_range<size_t>& r) {
            if (tbb::task::self().is_cancelled()) return;
            for (size_t i = r.begin(); i != r.end(); ++i) {
                if (pred(i)) {
                    ret = true;
                    tbb::task::self().cancel_group_execution();
                }
            }
        });
#else
        ret = parallel_reduce (first, last, false, 
            [pred](const range<size_t>& r)->bool {
                bool localret = false;
                for (auto i=r.begin(); i<r.end(); ++i) {
                    localret |= pred(i);
                }
                return localret;
            },
            std::bit_or<bool>()
        );
#endif

        return ret;
    }

} // end namespace
