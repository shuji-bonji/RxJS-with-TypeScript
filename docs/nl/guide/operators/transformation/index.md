---
description: Legt uit hoe u data in een stream kunt verwerken en transformeren met RxJS transformatieoperators, van eenvoudige transformaties zoals map, scan, mergeMap, switchMap en concatMap tot asynchrone transformaties, buffering en windowing. Praktische patronen die profiteren van TypeScript's typeveiligheid worden geïntroduceerd met overvloedige codevoorbeelden.
---

# Transformatieoperators

Transformatieoperators worden gebruikt om data binnen de RxJS-pipeline te transformeren en verwerken.
Door waarden naar nieuwe vormen te transformeren, maken ze flexibelere en krachtigere controle over reactieve datastromen mogelijk.


## Lijst van operators
### ◾ Eenvoudige waardetransformaties

|Operator|Beschrijving|
|---|---|
|[map](./map)|Pas een transformatiefunctie toe op elke waarde|

### ◾ Accumulatie

|Operator|Beschrijving|
|---|---|
|[scan](./scan)|Genereer waarden cumulatief|
|[reduce](./reduce)|Voer alleen het eindresultaat van accumulatie uit|

### ◾ Paar en groepering

|Operator|Beschrijving|
|---|---|
|[pairwise](./pairwise)|Verwerk twee opeenvolgende waarden in paren|
|[groupBy](./groupBy)|Groepeer waarden op basis van een sleutel|

### ◾ Asynchrone transformatie

|Operator|Beschrijving|
|---|---|
|[mergeMap](./mergeMap) |Transformeer elke waarde naar een Observable en voeg parallel samen|
|[switchMap](./switchMap) |Schakel naar de nieuwste Observable|
|[concatMap](./concatMap) |Voer elke Observable sequentieel uit|
|[exhaustMap](./exhaustMap) |Negeer nieuwe invoer tijdens uitvoering|
|[expand](./expand) |Breid resultaten recursief uit|

### ◾ Batchverwerking

|Operator|Beschrijving|
|---|---|
|[buffer](./buffer) |Batch waarden op de timing van een andere Observable|
|[bufferTime](./bufferTime) |Batch waarden op regelmatige intervallen|
|[bufferCount](./bufferCount) |Batch waarden op gespecificeerd aantal|
|[bufferWhen](./bufferWhen) |Buffering met dynamisch gecontroleerde eindvoorwaarden|
|[bufferToggle](./bufferToggle) |Buffering met onafhankelijke controle van start en einde|
|[windowTime](./windowTime) |Splits in sub-Observables op regelmatige intervallen|


## Praktische transformatiepatronen

In echte applicaties is de volgende verwerking mogelijk door transformatieoperators te combineren:

- Invoervalidatie en feedback
- Optimale controle van asynchrone API-verzoeken
- Data-vormgeving, aggregatie en normalisatie
- Batchverwerking en groepering van gebeurtenisstreams

👉 Voor meer informatie: [Praktische transformatiepatronen](./practical-use-cases)

## 🚨 Opmerkingen

Om veelvoorkomende fouten bij het gebruik van transformatieoperators te vermijden, zie ook:

- **[Bijwerkingen in map](/nl/guide/anti-patterns/common-mistakes#5-side-effects-in-map)** - Gebruik `map` als een pure functie
- **[Ongepaste operatorselectie](/nl/guide/anti-patterns/common-mistakes#12-inappropriate-operator-selection)** - Correct gebruik van hogere-orde operators
