package xiangshan

sealed trait BoundReplacementPolicy
object BoundReplacementPolicy {
  case object Disabled extends BoundReplacementPolicy
  case object SecondChance extends BoundReplacementPolicy
  case object SRRIP extends BoundReplacementPolicy
  case object BIP extends BoundReplacementPolicy
}

case class BoundReplacementParams(policy: BoundReplacementPolicy = BoundReplacementPolicy.Disabled) {
  require(policy == BoundReplacementPolicy.Disabled || policy == BoundReplacementPolicy.SecondChance,
    s"Bound replacement policy $policy is not implemented")
  def enabled: Boolean = policy == BoundReplacementPolicy.SecondChance
  // ABI[7:0], policy[15:8], state bits[23:16], install[31:24], hit[39:32].
  // Capabilities[47:40]: bit0 Mem query, bit1 Jump query, bit2 read-only R.
  def info: BigInt = if (enabled) BigInt("070101010102", 16) else BigInt(2)
}
