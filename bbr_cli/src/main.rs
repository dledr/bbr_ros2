mod cli;

use cli::param;
// use exitfailure::ExitFailure;
// use failure::ResultExt;

use structopt::{StructOpt};
use rusqlite::{Result};

fn main() -> Result<()> {
    let opt = param::Opt::from_args();
    // println!("{:?}", opt);
    param::Opt::handle(opt)?;
    Ok(())
}