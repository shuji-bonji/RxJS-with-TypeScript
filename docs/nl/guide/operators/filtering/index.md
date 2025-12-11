---
description: "RxJS filteringsoperators filteren gegevens uit streams op basis van voorwaarden of tijd. filter, take, skip, debounceTime, throttleTime, distinct, first, last en meer - deze gids biedt selectierichtlijnen per gebruik en praktische voorbeelden."
---

# Filteringsoperators

RxJS filteringsoperators zijn essentiële tools om alleen de benodigde gegevens uit een stream te selecteren en onnodige gegevens niet door te laten.
Hierdoor verbeteren de efficiëntie en prestaties van de applicatie aanzienlijk.

Filteringsoperators zijn een groep RxJS-operators die waarden in een stream selecteren en alleen waarden doorlaten die aan bepaalde voorwaarden voldoen.
Door de gegevensstroom te controleren en alleen de benodigde waarden te verwerken, kunnen efficiënte gegevensverwerkingspijplijnen worden gebouwd.


## Operatorlijst
### ◾ Basis filteringsoperators

| Operator | Beschrijving |
|:---|:---|
| [filter](./filter) | Laat alleen waarden door die aan een voorwaarde voldoen |
| [take](./take) | Haalt alleen de eerste opgegeven aantal waarden op |
| [takeLast](./takeLast) | Haalt de laatste opgegeven aantal waarden op |
| [takeWhile](./takeWhile) | Haalt waarden op zolang aan een voorwaarde wordt voldaan |
| [skip](./skip) | Slaat de eerste opgegeven aantal waarden over |
| [skipLast](./skipLast) | Slaat de laatste opgegeven aantal waarden over |
| [skipWhile](./skipWhile) | Slaat waarden over zolang aan een voorwaarde wordt voldaan |
| [skipUntil](./skipUntil) | Slaat waarden over totdat een andere Observable wordt geactiveerd |
| [first](./first) | Haalt de eerste waarde of de eerste waarde die aan een voorwaarde voldoet op |
| [last](./last) | Haalt de laatste waarde of de laatste waarde die aan een voorwaarde voldoet op |
| [elementAt](./elementAt) | Haalt de waarde op de opgegeven index op |
| [find](./find) | Vindt de eerste waarde die aan een voorwaarde voldoet |
| [findIndex](./findIndex) | Haalt de index van de eerste waarde die aan een voorwaarde voldoet op |
| [ignoreElements](./ignoreElements) | Negeert alle waarden en laat alleen complete/error door |


### ◾ Tijdgebaseerde filteringsoperators

| Operator | Beschrijving |
|:---|:---|
| [debounceTime](./debounceTime) | Geeft de laatste waarde vrij als er gedurende de opgegeven tijd geen invoer is |
| [throttleTime](./throttleTime) | Laat de eerste waarde door en negeert nieuwe waarden gedurende de opgegeven tijd |
| [auditTime](./auditTime) | Geeft de laatste waarde na de opgegeven tijd vrij |
| [audit](./audit) | Geeft de laatste waarde vrij met een aangepaste Observable om de periode te regelen |
| [sampleTime](./sampleTime) | Neemt samples van de nieuwste waarde op opgegeven tijdsintervallen |


### ◾ Voorwaardegebaseerde filteringsoperators

| Operator | Beschrijving |
|:---|:---|
| [distinct](./distinct) | Verwijdert alle dubbele waarden (geeft alleen unieke waarden) |
| [distinctUntilChanged](./distinctUntilChanged) | Verwijdert opeenvolgende dubbele waarden |
| [distinctUntilKeyChanged](./distinctUntilKeyChanged) | Detecteert alleen wijzigingen in specifieke eigenschappen |


## Praktische use cases

- In [Praktische use cases](./practical-use-cases.md) worden praktijkvoorbeelden geïntroduceerd die meerdere filteringsoperators combineren (real-time zoeken, oneindige scroll, enz.).
