---
description: "Gli operatori di trasformazione di RxJS vengono utilizzati per trasformare e processare i dati all'interno delle pipeline. Dalla semplice trasformazione ai pattern asincroni, buffering e windowing con map, scan, mergeMap, switchMap, concatMap, introduce pattern pratici sfruttando la type-safety di TypeScript con esempi di codice completi."
---

# Operatori di Trasformazione

Gli operatori di trasformazione vengono utilizzati per trasformare e processare i dati all'interno delle pipeline di RxJS.
Trasformando i valori in nuove forme, è possibile controllare i flussi di dati reattivi in modo più flessibile e potente.


## Elenco degli Operatori
### ◾ Trasformazione Semplice dei Valori

|Operatore|Descrizione|
|---|---|
|[map](./map)|Applica una funzione di trasformazione a ciascun valore|

### ◾ Elaborazione Cumulativa

|Operatore|Descrizione|
|---|---|
|[scan](./scan)|Genera valori cumulativi|
|[reduce](./reduce)|Emette solo il risultato cumulativo finale|

### ◾ Elaborazione di Coppie e Raggruppamento

|Operatore|Descrizione|
|---|---|
|[pairwise](./pairwise)|Elabora due valori consecutivi come coppia|
|[groupBy](./groupBy)|Raggruppa i valori in base alla chiave|

### ◾ Trasformazione Asincrona

|Operatore|Descrizione|
|---|---|
|[mergeMap](./mergeMap) |Trasforma ciascun valore in Observable e li combina in parallelo|
|[switchMap](./switchMap) |Passa all'Observable più recente|
|[concatMap](./concatMap) |Esegue ciascun Observable in sequenza|
|[exhaustMap](./exhaustMap) |Ignora nuovi input mentre è in corso un'esecuzione|
|[expand](./expand) |Espande i risultati ricorsivamente|

### ◾ Elaborazione Batch

|Operatore|Descrizione|
|---|---|
|[buffer](./buffer) |Raggruppa i valori al timing di un altro Observable|
|[bufferTime](./bufferTime) |Raggruppa i valori a intervalli di tempo regolari|
|[bufferCount](./bufferCount) |Raggruppa per numero specificato|
|[bufferWhen](./bufferWhen) |Buffering con controllo dinamico della condizione di fine|
|[bufferToggle](./bufferToggle) |Buffering con controllo indipendente di inizio e fine|
|[windowTime](./windowTime) |Divide in sub-Observable a intervalli di tempo regolari|


## Pattern di Trasformazione Pratici

Nelle applicazioni reali, combinando gli operatori di trasformazione
è possibile realizzare le seguenti elaborazioni.

- Validazione dell'input e feedback
- Controllo ottimale delle richieste API asincrone
- Formattazione, aggregazione e normalizzazione dei dati
- Elaborazione batch e raggruppamento dei flussi di eventi

👉 Per maggiori dettagli: consulta [Pattern di Trasformazione Pratici](./practical-use-cases).

## 🚨 Note

Per evitare errori comuni nell'uso degli operatori di trasformazione, consultare anche quanto segue.

- **[Effetti collaterali in map](/it/guide/anti-patterns/common-mistakes#5-map-での副作用)** - Usa `map` come funzione pura
- **[Selezione inappropriata dell'operatore](/it/guide/anti-patterns/common-mistakes#12-不適切なオペレーター選択)** - Uso appropriato degli operatori di ordine superiore
