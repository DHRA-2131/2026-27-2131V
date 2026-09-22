class Field{
    public:
        //Interior of the field is 140.5 in x 140.5 in
        //Format: {x: {low, high}, y: {low, high}}
        //(0, 0) is the left corner of the red side
        static constexpr double obstacles[4][2][2] = {
            {{9,13},{0, 4}}, //Match loaders
            {{20,25.5},{18.5,24}}, //Scoring pins
            {{18.5,24},{20,25.5}},
            {{67.375, 73.125},{67.375, 73.125}} //Center pin
        };
        //Format: {red: {{x, y}, {x, y}}, blue: {{x, y}, {x, y}}}
        static constexpr double match_loaders[2][2][2] = {
            {{11, 0}, {129.5, 0}},
            {{11, 140.5}, {129.5, 140.5}}
        };
        static constexpr double center_pin[2] = {70.25, 70.25};
};  