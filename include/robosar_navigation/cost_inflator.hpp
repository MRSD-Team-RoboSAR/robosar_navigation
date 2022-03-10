// Created by Indraneel on 8/03/22

#ifndef COST_INFLATOR_HPP
#define COST_INFLATOR_HPP

#include <assert.h>
#include <ros/console.h>

// TODO @indraneel : Right now inflates the entire map
// In the future can start inflating only a part of the map

static const unsigned char NO_INFORMATION = 255;
static const unsigned char LETHAL_OBSTACLE = 254;
static const unsigned char INSCRIBED_INFLATED_OBSTACLE = 253;
static const unsigned char FREE_SPACE = 0;

/**
 * @class CellData
 * @brief Storage for cell information used during obstacle inflation
 */
class CellData
{
public:
  /**
   * @brief  Constructor for a CellData objects
   * @param  i The index of the cell in the cost map
   * @param  x The x coordinate of the cell in the cost map
   * @param  y The y coordinate of the cell in the cost map
   * @param  sx The x coordinate of the closest obstacle cell in the costmap
   * @param  sy The y coordinate of the closest obstacle cell in the costmap
   * @return
   */
  CellData(double i, unsigned int x, unsigned int y, unsigned int sx, unsigned int sy) :
      index_(i), x_(x), y_(y), src_x_(sx), src_y_(sy)
  {
  }
  unsigned int index_;
  unsigned int x_, y_;
  unsigned int src_x_, src_y_;
};


class CostInflator  {

public:
    CostInflator() : seen_(NULL),cached_costs_(NULL),cached_distances_(NULL),initialised(false),inflate_unknown_(false) {


    }

    ~CostInflator() {

        if(seen_!=NULL)
            delete[] seen_;
        clearCaches();
        initialised = false;
    }

    void initialiseInflator(unsigned int sx,unsigned int sy,unsigned int cell_radius_,double res, double in_radius, double scaling_factor)
    {   
        resolution_ = res;
        size_x = sx;
        size_y = sy;
        if(seen_!=NULL)
            delete[] seen_;
        clearCaches();
        seen_ = new bool[size_x*size_y];

        // Set boundaries of inflation
        min_i = 0;
        min_j = 0;
        max_i = int(size_x);
        max_j = int(size_y);

        weight_ = scaling_factor;
        inscribed_radius_ = in_radius;
        cell_inflation_radius_ = cell_radius_;
        computeCaches();

        initialised = true;
    }


    void inflateCosts(unsigned char* costmap_)
    {
        if (cell_inflation_radius_ == 0 || !initialised)
        {
            ROS_WARN("Aborting inflate costs!");
            return;
        }

        ROS_ASSERT_MSG(inflation_cells_.empty(),"Inflation list must be empty at the beginning!");

        memset(seen_, false, size_x * size_y * sizeof(bool));

    
         // Initialise priority queue
         // Start with lethal obstacles: by definition distance is 0.0
        std::vector<CellData>& obs_bin = inflation_cells_[0.0];
        for (int j = min_j; j < max_j; j++)
        {
            for (int i = min_i; i < max_i; i++)
            {
                int index =  j*size_x + i;
                unsigned char cost = costmap_[index];
                if (cost == LETHAL_OBSTACLE)
                {
                    obs_bin.push_back(CellData(index, i, j, i, j));
                }
            }
        }

        // Process cells by increasing distance
         std::map<double, std::vector<CellData> >::iterator bin;
        for (bin = inflation_cells_.begin(); bin != inflation_cells_.end(); ++bin)
        {
            for (int i = 0; i < bin->second.size(); ++i)
            {
                // process all cells at distance dist_bin.first
                const CellData& cell = bin->second[i];
                unsigned int index = cell.index_;
                
                // ignore if already visited
                if (seen_[index])
                    continue;
                seen_[index] = true;

                unsigned char new_cost = costLookup(cell.x_,cell.y_, cell.src_x_, cell.src_y_);
                unsigned char old_cost = costmap_[index];

                if (old_cost == NO_INFORMATION && (inflate_unknown_ ? (new_cost > FREE_SPACE) : (new_cost >= INSCRIBED_INFLATED_OBSTACLE)))
                    costmap_[index] = new_cost;
                else
                    costmap_[index] = std::max(old_cost, new_cost);

                // Add neighbours to inflation list
                enqueue(index - 1, cell.x_ - 1, cell.y_, cell.src_x_, cell.src_y_);
                enqueue(index - size_x, cell.x_, cell.y_ - 1, cell.src_x_, cell.src_y_);
                enqueue(index + 1, cell.x_ + 1, cell.y_, cell.src_x_, cell.src_y_);
                enqueue(index + size_x, cell.x_, cell.y_ + 1, cell.src_x_, cell.src_y_);
            }
        }

        
        inflation_cells_.clear();
    }

    inline unsigned char costLookup(int mx, int my, int src_x, int src_y)
    {
        unsigned int dx = abs(mx - src_x);
        unsigned int dy = abs(my - src_y);
        return cached_costs_[dx][dy];
    }

     inline double distanceLookup(int mx, int my, int src_x, int src_y)
    {
        unsigned int dx = abs(mx - src_x);
        unsigned int dy = abs(my - src_y);
        return cached_distances_[dx][dy];
    }

     /** @brief  Given a distance, compute a cost.
     * @param  distance The distance from an obstacle in cells
     * @return A cost value for the distance */
    virtual inline unsigned char computeCost(double distance) const
    {
        unsigned char cost = 0;
        if (distance == 0)
            cost = LETHAL_OBSTACLE;
        else if (distance * resolution_ <= inscribed_radius_)
            cost = INSCRIBED_INFLATED_OBSTACLE;
        else
        {
            // make sure cost falls off by Euclidean distance
            double euclidean_distance = distance * resolution_;
            double factor = exp(-1.0 * weight_ * (euclidean_distance - inscribed_radius_));
            cost = (unsigned char)((INSCRIBED_INFLATED_OBSTACLE - 1) * factor);
        }
        return cost;
    }

    /**
     *  Caches two operations (both possible because distance is in terms of cell distance and coordinates are cell coordinates)
     * Cost calculation of a cell within inflation radius
     * Euclidean distance between two cells
     * 
     * */
    void computeCaches() {

        if (cell_inflation_radius_ == 0)
            return;

        cached_costs_ = new unsigned char*[cell_inflation_radius_ + 2];
        cached_distances_ = new double*[cell_inflation_radius_ + 2];

        for (unsigned int i = 0; i <= cell_inflation_radius_ + 1; ++i)
        {
            cached_costs_[i] = new unsigned char[cell_inflation_radius_ + 2];
            cached_distances_[i] = new double[cell_inflation_radius_ + 2];
            for (unsigned int j = 0; j <= cell_inflation_radius_ + 1; ++j)
            {
                cached_distances_[i][j] = hypot(i, j);
            }
        }

        for (unsigned int i = 0; i <= cell_inflation_radius_ + 1; ++i)
        {
            for (unsigned int j = 0; j <= cell_inflation_radius_ + 1; ++j)
            {
                cached_costs_[i][j] = computeCost(cached_distances_[i][j]);
            }
        }
    } 

    void clearCaches() {
        
        if (cached_distances_ != NULL)
        {
            for (unsigned int i = 0; i <= cell_inflation_radius_ + 1; ++i)
            {
                if (cached_distances_[i])
                    delete[] cached_distances_[i];
            }
            if (cached_distances_)
                delete[] cached_distances_;
            cached_distances_ = NULL;
        }

        if (cached_costs_ != NULL)
        {
            for (unsigned int i = 0; i <= cell_inflation_radius_ + 1; ++i)
            {
            if (cached_costs_[i])
                delete[] cached_costs_[i];
            }
            delete[] cached_costs_;
            cached_costs_ = NULL;
        }
    }

    inline void enqueue(unsigned int index, unsigned int mx, unsigned int my,
                                    unsigned int src_x, unsigned int src_y)
    {
        if (mx>0 && mx<size_x-1  && my>0 && my<size_y-1 && !seen_[index] )
        {
            // we compute our distance table one cell further than the inflation radius dictates so we can make the check below
            double distance = distanceLookup(mx, my, src_x, src_y);

            // we only want to put the cell in the list if it is within the inflation radius of the obstacle point
            if (distance > cell_inflation_radius_)
                return;

            // push the cell data onto the inflation list and mark
            inflation_cells_[distance].push_back(CellData(index, mx, my, src_x, src_y));
        }
    }


private:
    // pseudo priority queue of cells wrt distance from obstacles
    std::map<double, std::vector<CellData> > inflation_cells_;
    unsigned int cell_inflation_radius_;
    double inscribed_radius_;
    bool inflate_unknown_;
    bool* seen_;
    unsigned char** cached_costs_;
    double** cached_distances_;
    bool initialised;
    int min_i,min_j,max_i,max_j;
    unsigned int size_x,size_y;
    double resolution_;
    double weight_;
};

#endif