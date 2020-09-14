#include "utils.h"

/*
std::vector<double> BuiltTimepointsList(PlotDataMapRef &data)
{
    {
        std::vector<size_t> index_num( data.numeric.size(), 0);

        std::vector<double> out;

        bool loop = true;

        const double MAX = std::numeric_limits<double>::max();

        while(loop)
        {
            double min_time = MAX;
            double prev_time = out.empty() ? -MAX : out.back();
            loop = false;
            size_t count = 0;
            for(const auto& it: data.numeric)
            {
                const auto& plot = it.second;
                out.reserve(plot.size());

                size_t index = index_num[count];

                while ( index < plot.size() && plot.at(index).x <= prev_time   )
                {
                    index++;
                }
                if( index >= plot.size() )
                {
                    count++;
                    continue;
                }
                else{
                    loop = true;
                }
                index_num[count] = index;
                double time_val = plot.at(index).x;
                min_time = std::min( min_time, time_val);
                count++;
            }
            if( min_time < MAX)
            {
                out.push_back( min_time );
            }
        }
        return out;
    }
}
*/
