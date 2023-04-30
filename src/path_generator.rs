#[allow(unused)]
use micromath::F32Ext as _;

pub struct PathGenerator<const MAX_WAY_POINTS: usize = 256> {
    current_position: u32,
    target_position: u32,
    way_point: usize,
    steps: usize,
    way_points: [u32; MAX_WAY_POINTS],
    max_acceleration: f32,
    max_velocity: f32,
}

impl<const MAX_WAY_POINTS: usize> PathGenerator<MAX_WAY_POINTS> {
    pub fn new(initial_position: u32, max_acceleration: f32, max_velocity: f32) -> Self {
        Self { current_position: initial_position, target_position: initial_position, way_point: 0, steps: 0, way_points: [0u32; MAX_WAY_POINTS], max_acceleration, max_velocity }
    }

    pub fn is_moving(&self) -> bool {
        self.current_position != self.target_position
    }

    pub fn get_target_position(&self) -> u32 {
        self.target_position
    }

    pub fn force_stop(&mut self) {
        self.target_position = self.current_position
    }

    pub fn begin_move_to(&mut self, target_position: u32) {
        let direction = if target_position < self.current_position { -1 } else { 1 };
        let distance = if direction < 0 { self.current_position - target_position } else { target_position - self.current_position };
        let steps = (distance as f32 / self.max_velocity).ceil() as u32;
        let steps_to_accelerate = (self.max_velocity / self.max_acceleration).ceil() as u32;
        let (max_velocity, steps_to_accelerate) =  if steps < steps_to_accelerate*2 {
            (self.max_acceleration * steps as f32 / 2.0, steps / 2)
        } else {
            (self.max_velocity, steps_to_accelerate)
        };
        let steps = (distance as f32 / max_velocity + steps_to_accelerate as f32).ceil() as u32;
        //rprintln!("begin_move_to: {},{},{},{},{},{},{}", self.current_position, target_position, direction, distance, steps, max_velocity, steps_to_accelerate);

        for step in 0..steps {
            let offset = if step < steps_to_accelerate {
                ((step*step) as f32 *self.max_acceleration/2.0).floor() as u32
            } else if step < steps - steps_to_accelerate {
                let base = (steps_to_accelerate*steps_to_accelerate) as f32 *self.max_acceleration/2.0;
                (base + max_velocity * (step - steps_to_accelerate) as f32).ceil() as u32
            } else {
                let remaining_steps = steps - step;
                distance - ((remaining_steps*remaining_steps) as f32 * self.max_acceleration/2.0).floor() as u32
            };
            self.way_points[step as usize] = if direction < 0 { self.current_position - offset } else { self.current_position + offset };
        }
        self.way_point = 0;
        self.steps = steps as usize;
        self.target_position = target_position;
    }

    pub fn step_next(&mut self) -> u32 {
        let result = if self.way_point >= self.steps {
            self.current_position = self.target_position;
            self.target_position
        } else {
            let position = self.way_points[self.way_point];
            self.current_position = position;
            self.way_point += 1;
            position
        };
        //rprintln!("step_next: {}", result);
        result
    }

}
