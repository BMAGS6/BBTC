/*
 * This declaration is intentionally private to the IB0.2a smoke test. It
 * proves that a consumer can link against bbtc::bbtc without creating a
 * premature public header or pretending that a physics API already exists.
 */
int bbtc_private_build_anchor(void);

int
main(void)
{
    return bbtc_private_build_anchor();
}
