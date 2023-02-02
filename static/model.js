export default class Model {
    /** Construct a Model with given information (in this case a value) */
    constructor(minValue, value, maxValue) {
        this.value = value;
        this.minValue = minValue;
        this.maxValue = maxValue;
    }

    available(delta) {
        if (this.value + delta < this.minValue) { return false; }
        if (this.value + delta > this.maxValue) { return false; }

        return true;
    }

    adjust(delta) {
        if (this.available(delta)) {   // sanity check
            this.value += delta;
        }
    }

     // Critical ability to create a new model on demand from existing one, to trigger updates
     copy() {
        let m = new Model(this.minValue, this.value, this.maxValue);
        return m;
    }
}