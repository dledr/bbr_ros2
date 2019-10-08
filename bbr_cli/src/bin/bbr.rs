use bbr_cli::cli;
// use exitfailure::ExitFailure;
// use failure::ResultExt;
use std::error::Error;

fn main() -> Result<(), Box<dyn Error>> {
    let opt = cli::Opt::from_args();
    // println!("{:?}", opt);
    cli::Opt::handle(opt)?;
    Ok(())
}
