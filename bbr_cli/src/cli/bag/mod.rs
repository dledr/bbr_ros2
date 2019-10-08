use crate::api;

use std::error::Error;
use std::path::PathBuf;
use structopt::{StructOpt};

#[derive(Debug, StructOpt)]
pub enum Bag {
    /// Checks bagfile validity and authenticity
    Check(Check),

    /// Converts bagfile to bbr format
    Convert(Convert),

    /// Finalizes bagfile by ending all records
    Finalize(Finalize),
}

impl Bag { 
    pub fn handle(opt: Bag) -> Result<(), Box<dyn Error>> {
        match opt {
            Bag::Convert { 0: convert_opt} => {
                api::convert(convert_opt.input)?
            }
            _ => {
                println!("This subcommand is not yet implemented.");
            }
        }
        Ok(())
    }
}

#[derive(Debug, StructOpt)]
pub struct Check {

    /// Input file
    #[structopt(short, long)]
    input: PathBuf,

    // Degree of parallelism
    #[structopt(short, long, default_value = "1")]
    parallelism: u32,
}

#[derive(Debug, StructOpt)]
pub struct Convert {

    /// Input file
    #[structopt(short, long)]
    pub input: PathBuf,
}

#[derive(Debug, StructOpt)]
pub struct Finalize {

    /// Input file
    #[structopt(short, long)]
    input: PathBuf,
}