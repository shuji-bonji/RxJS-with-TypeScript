---
description: Utility operators zijn een groep hulpoperators in RxJS die verantwoordelijk zijn voor het controleren van bijwerkingen, vertragingsverwerking, abonnementbeheer, etc.
---

# Utility Operators

Utility operators in RxJS zijn een groep operators die verantwoordelijk zijn voor **hulpverwerking van streams (bijwerkingen, statuscontrole, UI-ondersteuning, etc.)** in plaats van het hoofddoel van dataconversie of filtering.

Op deze pagina worden operators gecategoriseerd op doel zoals hieronder weergegeven, en wordt een lijst gegeven om hun basisgebruik te bevestigen.
Voor gedetailleerd gebruik en praktische voorbeelden verwijzen we naar de respectievelijke pagina's of [Praktische use cases](./practical-use-cases.md).


## Lijst van operators (op doel)

### ◾ Bijwerkingen en statuscontrole

| Operator | Beschrijving | Vaak gecombineerd met |
|--------------|------|------------------|
| [tap](./tap.md) | Voer bijwerkingen uit zonder waarden te wijzigen (logoutput, UI-updates, etc.) | `map`, `switchMap` |
| [finalize](./finalize.md) | Voer opschoonverwerking uit wanneer de stream eindigt | `tap`, `catchError` |


### ◾ Timing en vertragingscontrole

| Operator | Beschrijving | Vaak gecombineerd met |
|--------------|------|------------------|
| [delay](./delay.md) | Vertraag de uitgifte van elke waarde met een gespecificeerde tijd | `tap`, `concatMap` |
| [timeout](./timeout.md) | Genereer een fout als uitgifte een bepaalde tijd overschrijdt | `catchError`, `retry` |
| [takeUntil](./takeUntil.md) | Beëindig abonnement wanneer de gespecificeerde Observable melding geeft | `interval`, `fromEvent` |


### ◾ Beginwaarde, herhaling, array-conversie, etc.

| Operator | Beschrijving | Vaak gecombineerd met |
|--------------|------|------------------|
| [startWith](./startWith.md) | Geef een beginwaarde uit aan het begin van de stream | `scan`, `combineLatest` |
| [repeat](./repeat.md) | Hernieuw abonnement op de gehele stream na voltooiing | `tap`, `delay` |
| [retry](./retry.md) | Probeer opnieuw bij fout | `catchError`, `switchMap` |
| [toArray](./toArray.md) | Geef alle waarden in de stream uit als een enkele array (bij voltooiing) | `concatMap`, `take` |


## Opmerkingen

- Verschil tussen `retry` en `repeat`:
  - `retry`: **Opnieuw proberen bij fout**
  - `repeat`: **Opnieuw proberen bij succesvolle voltooiing**
- `toArray` geeft geen waarde uit tenzij het voltooid is, dus het wordt vaak gebruikt met `take()` en dergelijke.
