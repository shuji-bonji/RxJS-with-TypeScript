---
description: "Les opérateurs utilitaires aident à contrôler les effets de bord, la gestion des délais et la gestion des abonnements dans RxJS. L'utilisation et les modèles pratiques de tap, delay, finalize, takeUntil, startWith, retry, repeat et d'autres opérateurs fréquemment utilisés en pratique sont expliqués."
---

# Opérateurs utilitaires

Les opérateurs utilitaires de RxJS sont un **groupe d'opérateurs qui ne sont pas destinés à la conversion ou au filtrage des données, mais au traitement auxiliaire du flux (effets de bord, contrôle de l'état, support de l'interface utilisateur, etc.)**.

Sur cette page, les opérateurs sont classés par objectif comme suit, et une liste est fournie pour identifier leurs utilisations de base.
Pour une utilisation détaillée et des exemples pratiques, voir les pages individuelles ou les [Cas d'utilisation pratiques](./practical-use-cases.md).


## Liste des opérateurs (par fonction)

### ◾ Effets de bord et contrôle de l'état

| Opérateur | Description | Opérateurs fréquemment combinés |
|--------------|------|------------------|
| [tap](./tap.md) | Effectuer des effets de bord sans changer les valeurs (sortie de journal, mises à jour de l'interface utilisateur, etc.) | `map`, `switchMap` |
| [finalize](./finalize.md) | Effectuer un nettoyage à la fin du flux | `tap`, `catchError` |


### ◾ Contrôle du timing et des délais

| Opérateur | Description | Opérateurs fréquemment combinés |
|--------------|------|------------------|
| [delay](./delay.md) | Retarder l'émission de chaque valeur pour un temps spécifié | `tap`, `concatMap` |
| [timeout](./timeout.md) | Une erreur se produit si l'émission dépasse un certain temps | `catchError`, `retry` |
| [takeUntil](./takeUntil.md) | L'abonnement se termine lorsque l'Observable spécifié notifie | `interval`, `fromEvent` |


### ◾ Valeur initiale, répétition, conversion en tableau, etc.

| Opérateur | Description | Opérateurs fréquemment combinés |
|--------------|------|------------------|
| [startWith](./startWith.md) | Valeur initiale émise au début du flux | `scan`, `combineLatest` |
| [repeat](./repeat.md) | Réabonner le flux entier après achèvement | `tap`, `delay` |
| [retry](./retry.md) | Réessai en cas d'erreur | `catchError`, `switchMap` |
| [toArray](./toArray.md) | Publie toutes les valeurs du flux dans un tableau (à la fin) | `concatMap`, `take` |


## Notes

- Différence entre `retry` et `repeat` :
  - `retry` : **réessai en cas d'erreur**
  - `repeat` : **réessai en cas de succès**
- `toArray` ne produit pas de valeur tant qu'il n'est pas terminé, donc il est couramment utilisé en conjonction avec `take()`, etc.
