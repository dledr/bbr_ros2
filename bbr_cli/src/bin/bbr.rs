use bbr_cli::{cli};
// use exitfailure::ExitFailure;
// use failure::ResultExt;

use rusqlite::{Result};

fn main() -> Result<()> {
    let opt = cli::Opt::from_args();
    // println!("{:?}", opt);
    cli::Opt::handle(opt)?;
    Ok(())
}