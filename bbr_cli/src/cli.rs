pub mod param {
    use std::path::PathBuf;
    use structopt::{clap::{ArgMatches, Shell}, StructOpt};

    extern crate rusqlite;
    use rusqlite::{Connection, Result};
    use rusqlite::NO_PARAMS;

    impl Opt {
        pub fn from_args() -> Opt { <Opt as StructOpt>::from_args() }
        // pub fn get_matches() -> Opt { <Opt as StructOpt>::get_matches() }
    }

    #[derive(Debug, StructOpt)]
    #[structopt(name = "bbr",
        about = "A command line interface for bbr")]
    pub struct Opt {
        #[structopt(subcommand)]
        pub cmd: Option<Command>,

        // /// enable debug mode
        // #[structopt(short, long)]
        // debug: bool,

        /// Enable verbose output
        #[structopt(short, long)]
        verbose: bool,
    }

    #[derive(Debug, StructOpt)]
    pub enum Command {
        /// Bagfile subcommand
        Bag(Bag),

        #[structopt(
            name = "completions",
            about = "Completions for bbr",
            usage = "Examples:
            $ source <(bbr completions bash)
            $ bbr completions fish | source")]
        Completions(Completions),
    }

    #[derive(Debug, StructOpt)]
    pub enum Bag {
        /// Checks bagfile validity and authenticity
        Check(Check),

        /// Converts bagfile to bbr format
        Convert(Convert),

        /// Finalizes bagfile by ending all records
        Finalize(Finalize),
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

    #[derive(Debug, StructOpt)]
    pub struct Completions {

        /// The shell to generate the script for
        pub shell: Shell,
    }

    pub fn convert(opt: Convert) -> Result<()> {
        let mut conn = Connection::open(opt.input)?;
        let tx = conn.transaction()?;

        tx.execute("
            ALTER TABLE topics ADD COLUMN
                bbr_nonce BLOB NOT NULL DEFAULT 0;",
            NO_PARAMS)?;
        tx.execute("
            ALTER TABLE topics ADD COLUMN
                bbr_digest BLOB NOT NULL DEFAULT 0;",
            NO_PARAMS)?;
        tx.execute("
            ALTER TABLE messages ADD COLUMN
                bbr_digest BLOB NOT NULL DEFAULT 0;",
            NO_PARAMS)?;
        tx.commit()
    }

    pub fn bag(opt: Bag) -> Result<()> {
        match opt {
            Bag::Convert { 0: convert_opt} => {
                convert(convert_opt)?
            }
            _ => {
                println!("This subcommand is not yet implemented.");
            }
        }
        Ok(())
    }

    pub fn completions(opt: Completions) {
        Opt::clap().gen_completions_to(
            "bbr",
            opt.shell,
            &mut std::io::stdout());
        std::process::exit(0)
    }
}