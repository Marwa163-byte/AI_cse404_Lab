import time

# ---------------- Minimax ----------------
def minimax(piles, is_maximizing_player):
    """
    Minimax algorithm for  Nim game
    (last move loses)
    """
    if all(pile == 0 for pile in piles):
        # Flip result compared to normal Nim
        return 1 if is_maximizing_player else -1

    if is_maximizing_player:
        best_score = float('-inf')
        for pile_idx in range(len(piles)):
            for take in range(1, piles[pile_idx] + 1):
                new_piles = piles[:]
                new_piles[pile_idx] -= take
                score = minimax(new_piles, False)
                best_score = max(best_score, score)
        return best_score
    else:
        best_score = float('inf')
        for pile_idx in range(len(piles)):
            for take in range(1, piles[pile_idx] + 1):
                new_piles = piles[:]
                new_piles[pile_idx] -= take
                score = minimax(new_piles, True)
                best_score = min(best_score, score)
        return best_score


def find_best_move_minimax(piles):
    best_score = float('-inf')
    best_move = None
    for pile_idx in range(len(piles)):
        for take in range(1, piles[pile_idx] + 1):
            new_piles = piles[:]
            new_piles[pile_idx] -= take
            score = minimax(new_piles, False)
            if score > best_score:
                best_score = score
                best_move = (pile_idx, take)
    return best_move


# ---------------- Misère Alpha-Beta ----------------
def minimax_alpha_beta(piles, is_maximizing_player, alpha=float('-inf'), beta=float('inf')):
    """
    Minimax with alpha-beta pruning for Misère Nim
    (last move loses)
    """
    if all(pile == 0 for pile in piles):
        return 1 if is_maximizing_player else -1

    if is_maximizing_player:
        max_eval = float('-inf')
        for pile_idx in range(len(piles)):
            if piles[pile_idx] == 0:
                continue
            for take in range(1, piles[pile_idx] + 1):
                new_piles = piles[:]
                new_piles[pile_idx] -= take
                eval_score = minimax_alpha_beta(new_piles, False, alpha, beta)
                max_eval = max(max_eval, eval_score)
                alpha = max(alpha, eval_score)
                if beta <= alpha:
                    break
            if beta <= alpha:
                break
        return max_eval
    else:
        min_eval = float('inf')
        for pile_idx in range(len(piles)):
            if piles[pile_idx] == 0:
                continue
            for take in range(1, piles[pile_idx] + 1):
                new_piles = piles[:]
                new_piles[pile_idx] -= take
                eval_score = minimax_alpha_beta(new_piles, True, alpha, beta)
                min_eval = min(min_eval, eval_score)
                beta = min(beta, eval_score)
                if beta <= alpha:
                    break
            if beta <= alpha:
                break
        return min_eval


def find_best_move_alpha_beta(piles):
    best_score = float('-inf')
    best_move = None
    alpha = float('-inf')
    beta = float('inf')

    for pile_idx in range(len(piles)):
        if piles[pile_idx] == 0:
            continue
        for take in range(1, piles[pile_idx] + 1):
            new_piles = piles[:]
            new_piles[pile_idx] -= take
            score = minimax_alpha_beta(new_piles, False, alpha, beta)
            if score > best_score:
                best_score = score
                best_move = (pile_idx, take)
            alpha = max(alpha, score)
    return best_move


# ---------------- Compare Both ----------------
if __name__ == "__main__":
    game_state = [3, 4, 5]  # Example state

    print(f"Current game state: {game_state}")

    # --- Minimax ---
    start = time.time()
    best_move_mini = find_best_move_minimax(game_state)
    mini_time = time.time() - start
    if best_move_mini:
        pile_idx, take = best_move_mini
        print(f" Minimax best move: Take {take} from pile {pile_idx+1}")
        print(f"Time taken (Minimax): {mini_time:.6f} seconds")

    # --- Alpha-Beta ---
    start = time.time()
    best_move_ab = find_best_move_alpha_beta(game_state)
    ab_time = time.time() - start
    if best_move_ab:
        pile_idx, take = best_move_ab
        print(f" Alpha-Beta best move: Take {take} from pile {pile_idx+1}")
        print(f"Time taken (Alpha-Beta): {ab_time:.6f} seconds")

    # --- Compare ---
    print("\n--- Comparison ---")
    print(f"Speedup: {(mini_time/ab_time):.2f}x faster with Alpha-Beta")
