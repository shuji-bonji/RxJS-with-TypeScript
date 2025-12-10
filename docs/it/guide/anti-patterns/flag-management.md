---
title: Proliferazione dei flag di gestione dello stato
description: "Spiega come migliorare il problema della proliferazione di 17 flag booleani nei progetti RxJS attraverso una progettazione reattiva. Introduce tecniche pratiche di refactoring che migliorano drasticamente la manutenibilità e la leggibilità rappresentando lo stato come Observable e integrando più flag con scan() e combineLatest()."
---

# Proliferazione dei flag di gestione dello stato

Anche nei progetti che hanno adottato RxJS, è comune vedere problemi con un gran numero di flag booleani sparsi nei componenti. Questo articolo spiega le cause e come risolvere il problema, basandosi su un esempio reale con ben 17 flag.

## Esempi reali del problema

Per prima cosa, esaminiamo del codice incontrato in situazioni reali. Ecco un tipico esempio di proliferazione di flag di gestione dello stato:

```typescript
class ProblematicComponent {
  // 17 flag presenti
  isLoading = false;
  isSaving = false;
  isDeleting = false;
  isEditing = false;
  hasError = false;
  isFormDirty = false;
  isFormValid = false;
  isDataLoaded = false;
  isUserAuthenticated = false;
  isModalOpen = false;
  isProcessing = false;
  isInitialized = false;
  isUpdating = false;
  isRefreshing = false;
  hasUnsavedChanges = false;
  isSubmitting = false;
  isValidating = false;

  save() {
    // Ramificazione complessa all'interno di subscribe
    this.apiService.save(this.data).subscribe({
      next: (result) => {
        if (this.isLoading && !this.isSaving) {
          if (this.isFormValid && this.isDataLoaded) {
            if (!this.hasError && !this.isProcessing) {
              // Elaborazione effettiva
              this.isSaving = false;
              this.hasUnsavedChanges = false;
            }
          }
        }
      },
      error: (err) => {
        this.isSaving = false;
        this.hasError = true;
        this.isProcessing = false;
      }
    });
  }
}
```

Questo tipo di codice si verifica **anche con RxJS implementato**. Questo pattern di gestione manuale di 17 flag controllati con ramificazioni condizionali complesse è problematico in termini di manutenibilità, leggibilità e testabilità.

## Perché i flag proliferano

Le ragioni alla base della proliferazione dei flag non sono solo tecniche, ma anche legate ai modelli di pensiero degli sviluppatori e al processo evolutivo dell'organizzazione. Di seguito vengono analizzate le cinque cause principali.

### Analisi strutturale delle cause

| Categoria | Sintomi specifici | Contesto |
|------------|------------|------|
| **① Persistenza del pensiero imperativo** | Più di 10 `isLoading`, `isSaving`, `isError`, ecc.<br>Un gran numero di guard come `if (this.isSaving) return;` | Invece degli stream RxJS, si usa il **controllo dei "flag di stato" imperativo** per la ramificazione logica.<br>Stato e side effect non possono essere separati, riducendo la leggibilità |
| **② Sottoutilizzo dello stato derivato** | Gestito direttamente nel componente con assegnazioni come `this.isLoaded = true;` | La derivazione dello stato potrebbe essere definita dichiarativamente usando `map` e `combineLatest` di Observable,<br>ma invece si compone lo stato manualmente |
| **③ Responsabilità ambigue nella progettazione dello stato** | Esistono più flag per lo stesso stato<br>(es: `isLoadingStart`, `isLoadingEnd`) | **Si tratta il cambiamento di stato come un comando**.<br>Ciò che dovrebbe essere integrato come "uno stato" è distribuito su più flag |
| **④ Ramificazione dello stream RxJS non organizzata** | Multipli `if` e `tap` concatenati in un `Observable`,<br>side effect e aggiornamenti di stato mescolati | Nessuna separazione delle responsabilità nella progettazione dello stream.<br>L'uso di `switchMap` e `catchError` è ambiguo |
| **⑤ Mancanza del layer ViewModel** | Manipolazione diretta di `this.isEditing`, `this.isSaved` nei componenti UI | Avere lo stato nei componenti elimina i benefici di RxJS |


## Causa principale: discrepanza nei modelli di pensiero

La causa principale della proliferazione dei flag è la **discrepanza tra i modelli di pensiero della programmazione imperativa e reattiva**. Se gli sviluppatori utilizzano RxJS con una mentalità imperativa, si verificano i seguenti problemi.

### Struttura transitoria

Molti progetti attraversano il seguente processo evolutivo e finiscono nell'inferno dei flag.

```
1. Aggiungere controlli con flag if per far funzionare le cose
   ↓
2. Introdurre RxJS successivamente
   ↓
3. La vecchia logica non può essere convertita in stream e si mescola
   ↓
4. L'inferno dei flag è completo
```

### Livelli misti di gestione dello stato

Lo stato in un'applicazione dovrebbe essere intrinsecamente gestito in tre livelli.

```
Applicazione
 ├── Stato View (isOpen, isLoading, formDirty)     ← nel componente
 ├── Stato Business (entity, filters, errors)      ← Livello di gestione dello stato
 └── Stato API (pending, success, error)           ← Stream RxJS
```

Se questi tre livelli non sono separati, lo stesso "flag" può avere **tre tipi diversi** di responsabilità. Gestire lo stato View e lo stato API allo stesso livello fa esplodere la complessità.

## Natura del problema: la "natura" dei flag

Il vero problema della proliferazione dei flag non è che ce ne sono "troppi", ma che i **flag sono variabili mutabili imperative**. Di seguito confrontiamo la differenza tra flag problematici e flag appropriati.

### ❌ Flag problematici: variabili mutabili imperative

```typescript
class BadComponent {
  // Questi non sono "stati" ma "comandi"
  isLoading = false;
  isSaving = false;
  hasError = false;

  save() {
    if (this.isSaving) return;        // Clausola guard necessaria
    this.isSaving = true;              // Modifica manuale

    this.api.save().subscribe({
      next: () => {
        this.isSaving = false;         // Reset manuale
        this.hasError = false;         // Gestire manualmente anche altri flag
      },
      error: () => {
        this.isSaving = false;         // Stesso processo in più punti
        this.hasError = true;
      }
    });
  }
}
```

> [!WARNING] Problemi
> - Lo stato è "procedurale" piuttosto che "dichiarativo"
> - Tempistica dispersa dei cambiamenti di stato
> - Coerenza tra i flag garantita manualmente dagli sviluppatori

### ✅ Flag appropriati: variabili reattive

```typescript
class GoodComponent {
  // Dichiarato come stream di stato
  private saveAction$ = new Subject<void>();

  readonly saveState$ = this.saveAction$.pipe(
    switchMap(() =>
      this.api.save().pipe(
        map(() => 'success' as const),
        catchError(() => of('error' as const)),
        startWith('loading' as const)
      )
    ),
    startWith('idle' as const),
    shareReplay(1)
  );

  // Stati derivati definiti anche dichiarativamente
  readonly isLoading$ = this.saveState$.pipe(
    map(state => state === 'loading')
  );

  readonly hasError$ = this.saveState$.pipe(
    map(state => state === 'error')
  );

  save() {
    this.saveAction$.next(); // Solo trigger dell'evento
  }
}
```

> [!TIP] Miglioramenti
> - Gli stati sono gestiti centralmente come "stream"
> - Le transizioni di stato sono definite dichiarativamente nella pipeline
> - La coerenza tra i flag è garantita automaticamente


## Criteri per la progettazione dei flag

Di seguito è riportato un elenco di criteri per determinare se il codice ha una progettazione di flag problematica. Usateli come riferimento durante la revisione e la progettazione del codice.

| Aspetto | ❌ Problematico | ✅ Non problematico |
|------|-----------|-----------|
| **Tipo** | `boolean` (mutabile) | `Observable<boolean>` / `Signal<boolean>` |
| **Come modificare** | Assegnazione diretta `flag = true` | Stream/derivato `state$.pipe(map(...))` |
| **Dipendenze** | Implicite (ordine del codice) | Esplicite (combineLatest, computed) |
| **Naming** | `xxxFlag`, `isXXX` (boolean) | `xxxState`, `canXXX`, `shouldXXX` |
| **Numero** | 10 o più boolean indipendenti | 1 stato + derivazioni multiple |


## Strategia di miglioramento

Per risolvere il problema della proliferazione dei flag, il refactoring può essere eseguito passo dopo passo nelle tre fasi seguenti.

### Step 1: Inventario degli stati

Prima di tutto, enumerare tutti i flag attuali e classificarli per responsabilità. Questo darà un'idea di quali flag possono essere integrati.

```typescript
// Enumerare i flag esistenti e classificare le responsabilità
interface StateInventory {
  view: string[];      // Controllo visualizzazione UI (isModalOpen, isEditing)
  business: string[];  // Logica di business (isFormValid, hasUnsavedChanges)
  api: string[];       // Stato comunicazione (isLoading, isSaving, hasError)
}
```

### Step 2: Enumerare lo stato

Successivamente, unire diversi flag booleani correlati come un singolo stato. Per esempio, `isLoading`, `isSaving` e `hasError` possono essere uniti come "stato della richiesta".

```typescript
// Unire più boolean in un unico stato
enum RequestState {
  Idle = 'idle',
  Loading = 'loading',
  Success = 'success',
  Error = 'error'
}

// Esempio di utilizzo
class Component {
  saveState: RequestState = RequestState.Idle;
  // isLoading, isSaving, hasError non sono più necessari
}
```

### Step 3: Reattivizzazione

Infine, gestire lo stato con Observable o Signal e definire lo stato derivato dichiarativamente. Questo garantisce automaticamente l'integrità dello stato.

```typescript
// Gestito da Observable o Signal
class ReactiveComponent {
  private readonly apiState$ = new BehaviorSubject<ApiState>({
    loading: false,
    saving: false,
    error: null
  });

  private readonly formState$ = this.form.valueChanges.pipe(
    map(() => ({
      dirty: this.form.dirty,
      valid: this.form.valid
    })),
    startWith({ dirty: false, valid: false })
  );

  // Integrato come ViewModel
  readonly vm$ = combineLatest([
    this.apiState$,
    this.formState$
  ]).pipe(
    map(([api, form]) => ({
      canSave: !api.saving && form.valid,
      showSpinner: api.loading || api.saving,
      showError: api.error !== null
    }))
  );
}
```


## Esempio di implementazione: refactoring di 17 flag

Questa sezione mostra il processo effettivo di refactoring del componente con 17 flag introdotto all'inizio in una progettazione reattiva. Confrontando i risultati Before/After, si possono vedere gli effetti dei miglioramenti.

### Before: gestione imperativa dei flag

Per prima cosa, rivediamo il codice problematico: 17 flag booleani in disordine, controllati da ramificazioni condizionali complesse.

```typescript
class LegacyComponent {
  isLoading = false;
  isSaving = false;
  isDeleting = false;
  isEditing = false;
  hasError = false;
  isFormDirty = false;
  isFormValid = false;
  isDataLoaded = false;
  isUserAuthenticated = false;
  isModalOpen = false;
  isProcessing = false;
  isInitialized = false;
  isUpdating = false;
  isRefreshing = false;
  hasUnsavedChanges = false;
  isSubmitting = false;
  isValidating = false;

  save() {
    if (!this.isLoading &&
        !this.isSaving &&
        this.isFormValid &&
        !this.hasError &&
        this.isDataLoaded) {
      this.isSaving = true;
      this.apiService.save().subscribe({
        next: () => {
          this.isSaving = false;
          this.hasUnsavedChanges = false;
        },
        error: () => {
          this.isSaving = false;
          this.hasError = true;
        }
      });
    }
  }
}
```

### After: gestione reattiva dello stato

Vediamo ora il codice migliorato: i 17 flag sono organizzati in tre stati di base (apiState$, formState$ e dataState$) e uno stato derivato (vm$).

```typescript
import { BehaviorSubject, combineLatest, EMPTY } from 'rxjs';
import { map, switchMap, catchError, startWith } from 'rxjs';

interface ApiState {
  loading: boolean;
  saving: boolean;
  deleting: boolean;
  error: string | null;
}

interface DataState {
  loaded: boolean;
  editing: boolean;
}

class RefactoredComponent {
  // Stato base gestito con Observable
  private readonly apiState$ = new BehaviorSubject<ApiState>({
    loading: false,
    saving: false,
    deleting: false,
    error: null
  });

  private readonly formState$ = this.form.valueChanges.pipe(
    map(() => ({
      dirty: this.form.dirty,
      valid: this.form.valid
    })),
    startWith({ dirty: false, valid: false })
  );

  private readonly dataState$ = new BehaviorSubject<DataState>({
    loaded: false,
    editing: false
  });

  // Integrato come ViewModel (stato derivato)
  readonly vm$ = combineLatest([
    this.apiState$,
    this.formState$,
    this.dataState$,
    this.authService.isAuthenticated$
  ]).pipe(
    map(([api, form, data, auth]) => ({
      // Stati derivati per la visualizzazione UI
      canSave: !api.saving && form.valid && data.loaded && auth,
      showSpinner: api.loading || api.saving || api.deleting,
      showError: api.error !== null,
      errorMessage: api.error,
      // Esporre anche stati individuali se necessario
      isEditing: data.editing,
      formDirty: form.dirty
    }))
  );

  save() {
    // Il controllo dello stato è fatto automaticamente dal ViewModel
    this.apiState$.next({
      ...this.apiState$.value,
      saving: true,
      error: null
    });

    this.apiService.save().pipe(
      catchError(error => {
        this.apiState$.next({
          ...this.apiState$.value,
          saving: false,
          error: error.message
        });
        return EMPTY;
      })
    ).subscribe(() => {
      this.apiState$.next({
        ...this.apiState$.value,
        saving: false
      });
    });
  }
}
```

### Utilizzo lato UI

Con la gestione reattiva dello stato, l'utilizzo lato UI diventa molto più semplice. Non è più necessario controllare individualmente più flag, basta ottenere le informazioni necessarie dal ViewModel.

```typescript
// Before: riferimento diretto a più flag
const isButtonDisabled =
  this.isLoading ||
  this.isSaving ||
  !this.isFormValid ||
  this.hasError ||
  !this.isDataLoaded;

// After: ottenere lo stato derivato dal ViewModel
this.vm$.subscribe(vm => {
  const isButtonDisabled = !vm.canSave;
  const showSpinner = vm.showSpinner;
  const errorMessage = vm.errorMessage;
});
```


## Importanza delle convenzioni di naming

Il naming è molto importante nella progettazione dei flag. Un naming appropriato permette di comprendere a colpo d'occhio le responsabilità, la natura e il ciclo di vita del flag. Al contrario, un naming ambiguo è fonte di confusione.

### ❌ Esempi di cattivo naming

Le seguenti pratiche di naming non sono chiare nelle intenzioni e riducono la manutenibilità.

```typescript
// Quale flag? Cosa innesca la modifica?
userFlag: boolean;
dataFlag: boolean;
checkFlag: boolean;

// È uno stato? Un'azione?
isProcess: boolean;  // In elaborazione? Elaborato?
```

### ✅ Esempi di buon naming

Un naming appropriato esprime chiaramente l'intento e la natura dello stato, usando Observable (suffisso `$`) e Signal per chiarire il tipo di stato (State, can, should).

```typescript
// Rappresentazione non ambigua dello stato
readonly userLoadState$: Observable<'idle' | 'loading' | 'loaded' | 'error'>;

// Lo stato derivato è chiaro anche negli intenti
readonly canSubmit$: Observable<boolean>;
readonly shouldShowSpinner$: Observable<boolean>;

// Esempio di utilizzo di Signal (disponibile in Angular, Preact, Solid.js, ecc.)
readonly userLoadState = signal<LoadState>('idle');
readonly canSubmit = computed(() =>
  this.userLoadState() === 'loaded' && this.formValid()
);
```


## Checklist diagnostica

Utilizza la seguente checklist per verificare se il tuo codice soffre del problema della proliferazione dei flag. Usala come riferimento durante la revisione del codice e la progettazione.

```markdown
## 🚨 Segnali di pericolo

- [ ] Più di 5 variabili boolean
- [ ] 3 o più istruzioni `if` annidate all'interno di `subscribe`
- [ ] Lo stesso flag è impostato in più punti
- [ ] 3 o più naming `isXXXing`
- [ ] Esiste un layer di gestione dello stato, ma il componente ha il proprio stato
- [ ] Naming multipli di `xxxFlag`
- [ ] La gestione degli errori è sparsa in ogni `subscribe`

## ✅ Segni di miglioramento

- [ ] Lo stato è gestito da `Observable` o `Signal`
- [ ] Gli stati derivati sono definiti con `map`/`computed`
- [ ] Le transizioni di stato sono descritte dichiarativamente
- [ ] Si applica il pattern ViewModel
- [ ] I nomi esprimono chiaramente l'intento
```

## Riepilogo

Questo articolo ha descritto le cause del problema della proliferazione dei flag nei progetti RxJS e come porvi rimedio. Infine, rivediamo alcuni punti importanti.

### Natura del problema

1. **Che ci sono 17 flag** ← Questo è un sintomo
2. **Che sono variabili mutabili imperative** ← Questa è l'essenza
3. **Le transizioni di stato non sono dichiarative** ← Questa è la causa
4. **Il naming è ambiguo (xxxFlag)** ← Questa è la fonte di confusione

### Direzione del miglioramento

Per risolvere il problema della proliferazione dei flag, sono necessarie le seguenti quattro trasformazioni:

- **Variabili boolean** → **Observable/Signal**
- **Assegnazione diretta** → **Pipeline di stream**
- **17 indipendenti** → **uno stato + stati derivati**
- **xxxFlag** → **xxxState$ / canXXX$**

### La cosa più importante

> [!IMPORTANT] Principio chiave
> "Lo stato è il risultato degli eventi, non è controllato direttamente dai flag"

L'introduzione di RxJS è un cambiamento di "pensiero", non di "sintassi". Se ci trasciniamo dietro il pensiero imperativo, non ci libereremo dell'inferno dei flag. Considerando lo stato come uno stream e progettando dichiarativamente, si migliorano manutenibilità, leggibilità e testabilità.


## Sezioni correlate

Per approfondire la conoscenza della gestione dei flag in questo articolo, consulta anche i seguenti articoli correlati.

- [Inferno dell'annidamento di if in subscribe](./subscribe-if-hell) - Come gestire correttamente le ramificazioni condizionali
- [Errori comuni e come affrontarli](./common-mistakes) - Dettagli sui 15 anti-pattern
- [Gestione degli errori](/it/guide/error-handling/strategies) - Strategie appropriate per la gestione degli errori
- [Subject e multicasting](/it/guide/subjects/what-is-subject) - Fondamenti della gestione dello stato

## Risorse di riferimento

Puoi approfondire la documentazione ufficiale di RxJS e le risorse di apprendimento.

- [Documentazione ufficiale RxJS](https://rxjs.dev/) - Riferimento API e guida ufficiale
- [Learn RxJS](https://www.learnrxjs.io/) - Esempi pratici per operatore
- [RxJS Marbles](https://rxmarbles.com/) - Per capire come funzionano gli operatori visivamente
