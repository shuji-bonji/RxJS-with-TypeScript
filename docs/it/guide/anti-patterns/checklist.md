---
description: "Checklist per evitare gli anti-pattern da verificare quando si scrive codice RxJS, che copre 16 best practice, tra cui la corretta disiscrizione, l'uso corretto di Subject, l'implementazione della gestione degli errori, la prevenzione dei memory leak e altri elementi essenziali per un codice reattivo robusto e manutenibile."
---

# Checklist per evitare gli anti-pattern

Utilizza questa checklist per verificare la presenza di anti-pattern nel tuo codice RxJS. Clicca su ogni voce per una spiegazione dettagliata e un esempio di codice.

## Voci di controllo

### 🔴 Evitare i problemi critici

| Controllo | Voce | Punti chiave |
|:---:|---|---|
| <input type="checkbox" /> | **[Subject pubblicato con asObservable()](./common-mistakes#1-esposizione-esterna-del-subject)** | Non esportare direttamente `Subject`, ma esporlo come Observable con `asObservable()`<br>Le modifiche allo stato sono possibili solo tramite metodi dedicati |
| <input type="checkbox" /> | **[Evitare subscribe annidati](./common-mistakes#2-subscribe-annidati-callback-hell)** | Non chiamare un altro `subscribe` all'interno di un `subscribe`<br>Appiattire con `switchMap`, `mergeMap`, `concatMap` ecc. |
| <input type="checkbox" /> | **[Gli stream infiniti devono essere sempre cancellati](./common-mistakes#3-dimenticare-unsubscribe-memory-leak)** | Annullare sempre la sottoscrizione degli stream infiniti, come i listener di eventi<br>Pattern `takeUntil` o gestione delle `Subscription` |
| <input type="checkbox" /> | **[Impostare esplicitamente shareReplay](./common-mistakes#4-uso-improprio-di-sharereplay)** | Utilizzare il formato `shareReplay({ bufferSize: 1, refCount: true })`<br>Abilitare il conteggio dei riferimenti per evitare memory leak |
| <input type="checkbox" /> | **[Evitare l'annidamento di if in subscribe](./subscribe-if-hell)** | Evitare condizionali complessi (annidamento di 3 o più livelli) all'interno di `subscribe`<br>Dichiarare con operatori quali `filter`, `iif` e `partition` |

### 🟡 Evitare i problemi che richiedono attenzione

| Controllo | Voce | Punti chiave |
|:---:|---|---|
| <input type="checkbox" /> | **[map è una funzione pura, gli effetti collaterali in tap](./common-mistakes#5-effetti-collaterali-in-map)** | Nessun cambiamento di stato o output di log all'interno di `map`<br>Gli effetti collaterali sono esplicitamente separati dall'operatore `tap` |
| <input type="checkbox" /> | **[Usare Cold/Hot appropriatamente](./common-mistakes#6-ignorare-differenze-cold-hot-observable)** | Convertire richieste HTTP ecc. in Hot con `shareReplay`<br>Decidere se eseguire per ogni sottoscrizione o condividere |
| <input type="checkbox" /> | **[Promise convertita con from](./common-mistakes#7-mischiare-promise-e-observable-in-modo-improprio)** | Non mischiare Promise e Observable<br>Convertire in Observable con `from()` per un'elaborazione unificata |
| <input type="checkbox" /> | **[Gli eventi ad alta frequenza sono controllati](./common-mistakes#8-ignorare-la-backpressure)** | Controllare l'input di ricerca con `debounceTime` e lo scroll con `throttleTime`<br>`distinctUntilChanged` per escludere i duplicati |

### 🔵 Miglioramento della qualità del codice

| Controllo | Voce | Punti chiave |
|:---:|---|---|
| <input type="checkbox" /> | **[Gestire correttamente gli errori](./common-mistakes#9-soppressione-degli-errori)** | Catturare gli errori con `catchError` e gestirli appropriatamente<br>Mostrare messaggi di errore comprensibili<br>Riprovare con `retry` / `retryWhen` se necessario |
| <input type="checkbox" /> | **[Rilasciare correttamente gli eventi DOM](./common-mistakes#10-leak-sottoscrizioni-eventi-dom)** | Annullare sempre la sottoscrizione di `fromEvent`<br>Annullare automaticamente con `takeUntil` quando il componente viene distrutto |
| <input type="checkbox" /> | **[Garantire la type safety](./common-mistakes#11-mancanza-di-type-safety-uso-eccessivo-di-any)** | Definire interfacce e alias di tipo<br>Parametri di tipo espliciti per `Observable<T>`<br>Sfruttare l'inferenza di tipo di TypeScript |
| <input type="checkbox" /> | **[Selezionare l'operatore appropriato](./common-mistakes#12-selezione-impropria-delloperatore)** | Ricerca: `switchMap`, parallelo: `mergeMap`<br>Sequenziale: `concatMap`, prevenire clic ripetuti: `exhaustMap` |
| <input type="checkbox" /> | **[RxJS non necessario per elaborazioni semplici](./common-mistakes#13-eccessiva-complessita)** | Il normale JavaScript è sufficiente per l'elaborazione di array ecc.<br>RxJS è usato per l'elaborazione asincrona e i flussi di eventi |
| <input type="checkbox" /> | **[Lo stato è gestito in modo reattivo](./common-mistakes#14-cambiamenti-di-stato-in-subscribe)** | Gestire lo stato con `BehaviorSubject` o `scan`<br>`subscribe` è usato come trigger finale |
| <input type="checkbox" /> | **[Scrivere test](./common-mistakes#15-mancanza-di-test)** | Test marble con `TestScheduler`<br>L'elaborazione asincrona può essere testata in modo sincrono |

## Utilizzo

### 1. Durante la revisione del codice

Dopo aver scritto nuovo codice, utilizza questa checklist per effettuare un'auto-revisione.

### 2. Durante le pull request

Includi questa checklist nel template delle pull request, in modo che tu e i revisori abbiate criteri comuni.

### 3. Revisione periodica

Utilizza regolarmente questa checklist per verificare la presenza di anti-pattern nella codebase esistente.

### 4. Condivisione con il team

Condividila con i membri del team per unificare le best practice di RxJS.

## Risorse correlate

- **[Errori comuni e come affrontarli](./common-mistakes)** - Spiegazioni dettagliate ed esempi di codice per ogni anti-pattern
- **[Raccolta di anti-pattern RxJS](./index)** - Elenco di anti-pattern e percorso di apprendimento
- **[Gestione degli errori](/it/guide/error-handling/strategies)** - Best practice per la gestione degli errori
- **[Metodologie di test](/it/guide/testing/unit-tests)** - Come testare il codice RxJS

## Suggerimenti per l'uso della checklist

1. **Non cercare di perfezionare tutte le voci in una volta**
   - Affronta prima i problemi critici (🔴)
   - Migliora passo dopo passo

2. **Stabilisci le priorità all'interno del team**
   - Adatta il livello di importanza in base alle caratteristiche del progetto
   - Crea checklist personalizzate

3. **Considera l'automazione**
   - Controlli automatizzati con strumenti di analisi statica come ESLint
   - Incorpora nella pipeline CI/CD

4. **Aggiorna regolarmente**
   - Aggiorna in base agli aggiornamenti di versione di RxJS
   - Rifletti le conoscenze acquisite dall'esperienza del team

---

**Importante**: Questa checklist non è una guida per scrivere codice perfetto, ma per evitare problemi comuni. Utilizzala in modo flessibile in base al contesto del tuo progetto.
