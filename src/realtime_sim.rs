use crate::fixed_wing::FixedWing;

pub struct RealtimeSim {
    fixed_wing: FixedWing,
}

impl RealtimeSim {
    pub async fn run(&self) -> ! {
        loop {}
    }
}
