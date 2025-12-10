---
description: "Una guida pratica per comprendere gli anti-pattern di RxJS e scrivere codice più robusto e manutenibile, spiegando sistematicamente i problemi e le soluzioni che si verificano frequentemente nello sviluppo, come l'uso improprio di Subject, i subscribe annidati, la ramificazione condizionale all'interno dei subscribe e la proliferazione di flag."
---

# Raccolta di anti-pattern RxJS

RxJS è una potente libreria di programmazione reattiva, ma se usata in modo non corretto, può essere un terreno fertile per i bug e ridurre la manutenibilità. In questa sezione presentiamo gli errori più comuni nell'uso di RxJS in TypeScript e le best practice per evitarli.

## Scopo di questa sezione

- **Prevenire i bug**: evitare i problemi di implementazione comprendendo in anticipo gli errori più comuni
- **Migliorare la manutenibilità**: imparare pattern di codice facili da leggere e testare
- **Ottimizzare le prestazioni**: imparare tecniche per evitare memory leak ed elaborazioni non necessarie

## Elenco di anti-pattern

Questa sezione tratta i seguenti 17 anti-pattern.

### 🔴 Problemi critici

Questi pattern possono avere un grave impatto sull'applicazione.

| Pattern | Problema | Impatto |
|---|---|---|
| **[Esposizione esterna del Subject](./common-mistakes#1-esposizione-esterna-del-subject)** | Esporre `Subject` così com'è e consentire chiamate esterne a `next()` | Imprevedibilità della gestione dello stato, difficoltà di debug |
| **[Subscribe annidati](./common-mistakes#2-subscribe-annidati-callback-hell)** | Chiamare più `subscribe` dentro `subscribe` | Callback hell, complicazioni nella gestione degli errori |
| **[Proliferazione di flag di gestione stato](./flag-management)** | 17 flag booleani per gestire lo stato, pensiero imperativo residuo | Leggibilità ridotta, manutenzione difficile, focolaio di bug |
| **[Nidificazione di if in subscribe](./subscribe-if-hell)** | Ramificazione condizionale complessa all'interno di `subscribe` (3 o più livelli di nidificazione) | Leggibilità ridotta, difficile da testare, viola il pensiero dichiarativo |
| **[Dimenticare unsubscribe](./common-mistakes#3-dimenticare-unsubscribe-memory-leak)** | Non annullare la sottoscrizione di stream infiniti | Memory leak, spreco di risorse |
| **[Uso improprio di shareReplay](./common-mistakes#4-uso-improprio-di-sharereplay)** | Usare `shareReplay` senza capire come funziona | Riferimenti a dati obsoleti, memory leak |

### 🟡 Problemi che richiedono attenzione

Questi possono rappresentare un problema in determinate situazioni.

| Pattern | Problema | Impatto |
|---|---|---|
| **[Effetti collaterali in map](./common-mistakes#5-effetti-collaterali-in-map)** | Cambiare stato nell'operatore `map` | Comportamento imprevedibile, difficile da testare |
| **[Ignorare Cold/Hot](./common-mistakes#6-ignorare-differenze-cold-hot-observable)** | Non considerare la natura dell'Observable | Esecuzione duplicata, comportamento inatteso |
| **[Mescolare Promise e Observable](./promise-observable-mixing)** | Non convertire correttamente tra Promise e Observable | Non annullabile, scarsa gestione degli errori |
| **[Ignorare la backpressure](./common-mistakes#8-ignorare-la-backpressure)** | Non controllare eventi ad alta frequenza | Degrado delle prestazioni, blocco dell'UI |

### 🔵 Problemi di qualità del codice

Non sono bug diretti, ma fattori che riducono la qualità del codice.

| Pattern | Problema | Impatto |
|---|---|---|
| **[Soppressione degli errori](./common-mistakes#9-soppressione-degli-errori)** | Non gestire correttamente gli errori | Difficoltà di debug, scarsa esperienza utente |
| **[Leak di eventi DOM](./common-mistakes#10-leak-sottoscrizioni-eventi-dom)** | Non rilasciare gli event listener DOM | Memory leak, scarse prestazioni |
| **[Mancanza di type safety](./common-mistakes#11-mancanza-di-type-safety-uso-eccessivo-di-any)** | Uso eccessivo di `any` | Errori a runtime, difficoltà di refactoring |
| **[Selezione impropria dell'operatore](./common-mistakes#12-selezione-impropria-delloperatore)** | Usare operatori non adatti allo scopo | Inefficienza, comportamento inatteso |
| **[Eccessiva complessità](./common-mistakes#13-eccessiva-complessita)** | Complicare processi che potrebbero essere scritti semplicemente | Leggibilità ridotta, difficile da mantenere |
| **[One-liner hell](./one-liner-hell)** | Miscuglio di definizioni di stream, trasformazioni e sottoscrizioni | Difficile da debuggare, difficile da testare, leggibilità ridotta |
| **[Cambiamenti di stato in subscribe](./common-mistakes#14-cambiamenti-di-stato-in-subscribe)** | Cambiare direttamente lo stato all'interno di `subscribe` | Difficile da testare, causa di bug |
| **[Mancanza di test](./common-mistakes#15-mancanza-di-test)** | Non scrivere test per il codice RxJS | Regressioni, difficoltà di refactoring |

## Processo di apprendimento

1. **[Errori comuni e come affrontarli](./common-mistakes)** per apprendere 15 anti-pattern in dettaglio
2. Per ogni anti-pattern, troverete codice di "cattivo esempio" e "buon esempio"
3. **[Checklist per evitare gli anti-pattern](./checklist)** per rivedere il proprio codice
4. Implementare le best practice e condividerle con il team

## Sezioni correlate

Dopo aver appreso gli anti-pattern, consultate anche le seguenti sezioni:

- **[Gestione degli errori](/it/guide/error-handling/strategies)** - Strategie appropriate per la gestione degli errori
- **[Metodologie di test](/it/guide/testing/unit-tests)** - Come testare il codice RxJS
- **[Comprendere gli operatori](/it/guide/operators/)** - Come scegliere l'operatore giusto

## Prossimi passi

1. Iniziate da **[Errori comuni e come affrontarli](./common-mistakes)** per imparare gli anti-pattern pratici e le loro soluzioni
2. Dopo l'apprendimento, usate la **[Checklist per evitare gli anti-pattern](./checklist)** per rivedere il codice attuale

---

**Importante**: Questi anti-pattern si trovano frequentemente nei progetti reali. Comprenderli presto vi aiuterà a scrivere codice RxJS di qualità.
