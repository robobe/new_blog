use std::process;

trait Animal {
    fn speak(&self);
}

struct Dog;
struct Cat;

impl Animal for Dog {
    fn speak(&self) {
        println!("Woof");
    }
}

impl Animal for Cat {
    fn speak(&self) {
        println!("Meow");
    }
}

fn create_animal(kind: &str) -> Result<Box<dyn Animal>, String> {
    match kind {
        "dog" => Ok(Box::new(Dog)),
        "cat" => Ok(Box::new(Cat)),
        _ => Err(format!("Unknown animal type: {}", kind)),
    }
}

fn main() {
    if let Err(err) = run() {
        eprintln!("Error: {err}");
        process::exit(1);
    }
}

fn run() -> Result<(), String> {
    let animal = create_animal("ccc")?;
    animal.speak();
    Ok(())
}
