---
title: Wildgroei van Statusbeheer Flags
description: "Methode om het probleem van 17 boolean flags in RxJS projecten te verbeteren met reactief ontwerp. Presenteert praktische refactoring-technieken die onderhoudbaarheid en leesbaarheid drastisch verbeteren door status uit te drukken met Observable en meerdere flags te integreren met scan() en combineLatest()."
# outline: deep
---

# Wildgroei van Statusbeheer Flags

Zelfs in projecten die RxJS hebben geïntroduceerd, zie je vaak het probleem van grote aantallen boolean flags in componenten. Dit artikel legt de oorzaak en verbetermethoden uit gebaseerd op een daadwerkelijk geval met 17 flags.

## Voorbeeld van het Probleem

Laten we eerst de code bekijken die in de praktijk werd aangetroffen. Dit is een typisch voorbeeld waarin statusbeheer flags woekeren.

```typescript
class ProblematicComponent {
  // 17 flags bestaan
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
    // Complexe vertakkingen binnen subscribe
    this.apiService.save(this.data).subscribe({
      next: (result) => {
        if (this.isLoading && !this.isSaving) {
          if (this.isFormValid && this.isDataLoaded) {
            if (!this.hasError && !this.isProcessing) {
              // Daadwerkelijke verwerking
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

Dit soort code ontstaat **zelfs met RxJS-introductie**. Dit patroon van handmatig beheren van 17 flags en controleren met complexe voorwaardelijke vertakkingen heeft problemen in onderhoudbaarheid, leesbaarheid en testbaarheid.

## Waarom Woekeren Flags

Achter de wildgroei van flags zijn niet alleen technische problemen, maar ook denkpatronen van ontwikkelaars en evolutieprocessen van organisaties betrokken. Hieronder analyseren we 5 belangrijke oorzaken.

### Structurele Analyse van Oorzaken

| Oorzaak Categorie | Specifiek Symptoom | Achtergrond |
|------------|------------|------|
| **① Overblijfsel van Imperatief Denken** | 10+ flags zoals `isLoading`, `isSaving`, `isError`<br>Grote hoeveelheden guards zoals `if (this.isSaving) return;` | Logica wordt vertakt met **imperatieve "statusflag" controle** in plaats van RxJS streams.<br>Status en bijwerkingen kunnen niet worden gescheiden, leesbaarheid daalt |
| **② Onbenut Afgeleide Status** | Beheer door directe toewijzing zoals `this.isLoaded = true;` aan componentzijde | Afleiding van status kan declaratief worden gedefinieerd met `map` of `combineLatest` van Observable,<br>maar status wordt handmatig gecomponeerd zonder dat te doen |
| **③ Vage Verantwoordelijkheid van Status Ontwerp** | Meerdere flags bestaan voor dezelfde status<br>(bijv: `isLoadingStart`, `isLoadingEnd`) | **Statusverandering wordt behandeld als commando**.<br>"Eén status" die geïntegreerd moet worden is verspreid over meerdere flags |
| **④ Ongeorganiseerde RxJS Stream Vertakking** | Meerdere `if` en `tap` zijn verbonden binnen één `Observable`,<br>bijwerkingen en statusupdates zijn vermengd | Verantwoordelijkheden scheiden van stream ontwerp lukt niet.<br>Gebruik van `switchMap` en `catchError` is vaag |
| **⑤ Gebrek aan ViewModel Laag** | Directe manipulatie van `this.isEditing`, `this.isSaved` in UI componenten | Door status in component te houden,<br>worden RxJS voordelen afgesneden |


## Grondoorzaak: Mismatch in Denkmodel

De grondoorzaak van flag wildgroei is de **mismatch tussen imperatief programmeren en reactief programmeren denkmodellen**. Als ontwikkelaars RxJS gebruiken met imperatief denken, ontstaan de volgende problemen.

### Overgangsstructuur

Veel projecten vallen in flag hell door het volgende evolutieproces.

```
1. Voeg if-flag controle toe om het te laten werken
   ↓
2. Introduceer RxJS later
   ↓
3. Oude logica kan niet worden gestreamd en vermengd raken
   ↓
4. Flag hell is compleet
```

### Statusbeheer Lagen zijn Vermengd

Status binnen een applicatie moet oorspronkelijk in 3 lagen worden beheerd.

```
Applicatie
 ├── View status (isOpen, isLoading, formDirty)     ← Binnen component
 ├── Business status (entity, filters, errors)      ← Statusbeheer laag
 └── API status (pending, success, error)           ← RxJS stream
```

Als deze 3 lagen niet zijn gescheiden, raken **3 verschillende soorten** "flags" met verschillende verantwoordelijkheden vermengd. Als View status en API status op hetzelfde niveau worden beheerd, explodeert de complexiteit.

## Essentie van het Probleem: "Eigenschappen" van Flags

Het echte probleem met flag wildgroei is niet "het grote aantal", maar dat **flags imperatieve mutable variabelen zijn geworden**. Hieronder vergelijken we het verschil tussen problematische flags en geschikte flags.

### ❌ Problematische Flags: Imperatieve Mutable Variabelen

```typescript
class BadComponent {
  // Deze zijn "commando's" geworden in plaats van "status"
  isLoading = false;
  isSaving = false;
  hasError = false;

  save() {
    if (this.isSaving) return;        // Guard clause nodig
    this.isSaving = true;              // Handmatig wijzigen

    this.api.save().subscribe({
      next: () => {
        this.isSaving = false;         // Handmatig resetten
        this.hasError = false;         // Andere flags ook handmatig beheren
      },
      error: () => {
        this.isSaving = false;         // Dezelfde verwerking op meerdere plaatsen
        this.hasError = true;
      }
    });
  }
}
```

> [!WARNING] Problemen
> - Status is "procedureel" in plaats van "declaratief"
> - Timing van statuswijzigingen is verspreid
> - Ontwikkelaar moet handmatig consistentie tussen flags garanderen

### ✅ Geschikte Flags: Reactieve Variabelen

```typescript
class GoodComponent {
  // Declareren als status stream
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

  // Afgeleide status ook declaratief definiëren
  readonly isLoading$ = this.saveState$.pipe(
    map(state => state === 'loading')
  );

  readonly hasError$ = this.saveState$.pipe(
    map(state => state === 'error')
  );

  save() {
    this.saveAction$.next(); // Alleen event activeren
  }
}
```

> [!TIP] Verbeteringen
> - Status centraal beheerd als "stream"
> - Statustransities declaratief gedefinieerd in pipeline
> - Consistentie tussen flags automatisch gegarandeerd


## Beoordelingscriteria voor Flag Ontwerp

Hieronder zijn criteria samengevat om te beoordelen of je code problematisch flag ontwerp heeft. Gebruik als referentie bij code review of ontwerp.

| Aspect | ❌ Problematisch | ✅ Geen Probleem |
|------|-----------|-----------|
| **Type** | `boolean` (mutable) | `Observable<boolean>` / `Signal<boolean>` |
| **Wijzigingsmethode** | Directe toewijzing `flag = true` | Stream/afleiding `state$.pipe(map(...))` |
| **Afhankelijkheid** | Impliciet (codevolgorde) | Expliciet (combineLatest, computed) |
| **Naamgeving** | `xxxFlag`, `isXXX` (boolean) | `xxxState`, `canXXX`, `shouldXXX` |
| **Aantal** | 10+ onafhankelijke booleans | 1 status + meerdere afgeleiden |


## Verbeteringsstrategie

Om het flag wildgroei probleem op te lossen, refactoren we stapsgewijs in de volgende 3 stappen.

### Step 1: Status Inventariseren

Eerst lijst je alle huidige flags op en classificeer ze naar verantwoordelijkheid. Hierdoor zie je welke flags kunnen worden geïntegreerd.

```typescript
// Lijst bestaande flags en classificeer verantwoordelijkheden
interface StateInventory {
  view: string[];      // UI weergavecontrole (isModalOpen, isEditing)
  business: string[];  // Bedrijfslogica (isFormValid, hasUnsavedChanges)
  api: string[];       // Communicatiestatus (isLoading, isSaving, hasError)
}
```

### Step 2: Status Enum-iseren

Vervolgens integreer je meerdere gerelateerde boolean flags als één status. Bijvoorbeeld, `isLoading`, `isSaving`, `hasError` kunnen allemaal worden geïntegreerd als "request status".

```typescript
// Integreer meerdere booleans als één status
enum RequestState {
  Idle = 'idle',
  Loading = 'loading',
  Success = 'success',
  Error = 'error'
}

// Gebruiksvoorbeeld
class Component {
  saveState: RequestState = RequestState.Idle;
  // isLoading, isSaving, hasError niet meer nodig
}
```

### Step 3: Reactiveren

Ten slotte beheer je status met Observable of Signal en definieer afgeleide status declaratief. Hierdoor wordt statusconsistentie automatisch gegarandeerd.

```typescript
// Beheren met Observable of Signal
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

  // Integreren als ViewModel
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


## Implementatievoorbeeld: Refactoring van 17 Flags

Hier laten we het daadwerkelijke refactoringproces zien van de component met 17 flags die in het begin werd geïntroduceerd naar reactief ontwerp. Door Before/After te vergelijken kun je het effect van de verbetering ervaren.

### Before: Imperatief Flag Beheer

Eerst bevestigen we de problematische code opnieuw. 17 boolean flags woekeren en worden gecontroleerd met complexe voorwaardelijke vertakkingen.

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

### After: Reactief Statusbeheer

Vervolgens kijken we naar de verbeterde code. 17 flags zijn georganiseerd in 3 basisstatussen (apiState$, formState$, dataState$) en 1 afgeleide status (vm$).

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
  // Beheer basisstatus met Observable
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

  // Integreren als ViewModel (afgeleide status)
  readonly vm$ = combineLatest([
    this.apiState$,
    this.formState$,
    this.dataState$,
    this.authService.isAuthenticated$
  ]).pipe(
    map(([api, form, data, auth]) => ({
      // Afgeleide status voor UI weergave
      canSave: !api.saving && form.valid && data.loaded && auth,
      showSpinner: api.loading || api.saving || api.deleting,
      showError: api.error !== null,
      errorMessage: api.error,
      // Publiceer ook individuele status indien nodig
      isEditing: data.editing,
      formDirty: form.dirty
    }))
  );

  save() {
    // ViewModel controleert automatisch status
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

### Gebruik aan UI-zijde

Door reactief statusbeheer wordt gebruik aan UI-zijde ook aanzienlijk eenvoudiger. Het is niet meer nodig om meerdere flags individueel te controleren, je hoeft alleen de benodigde informatie uit ViewModel te halen.

```typescript
// Before: Directe referentie naar meerdere flags
const isButtonDisabled =
  this.isLoading ||
  this.isSaving ||
  !this.isFormValid ||
  this.hasError ||
  !this.isDataLoaded;

// After: Haal afgeleide status uit ViewModel
this.vm$.subscribe(vm => {
  const isButtonDisabled = !vm.canSave;
  const showSpinner = vm.showSpinner;
  const errorMessage = vm.errorMessage;
});
```


## Belang van Naamgevingsconventies

Bij flag ontwerp is naamgeving zeer belangrijk. Door juiste naamgeving kun je de verantwoordelijkheid, eigenschappen en levenscyclus van die flag in één oogopslag begrijpen. Omgekeerd is vage naamgeving de oorzaak van verwarring.

### ❌ Slechte Naamgevingsvoorbeelden

De volgende naamgeving is onduidelijk in intentie en vermindert onderhoudbaarheid.

```typescript
// Wat voor flag? Wat triggert de verandering?
userFlag: boolean;
dataFlag: boolean;
checkFlag: boolean;

// Is het status? Is het actie?
isProcess: boolean;  // Verwerken? Verwerkt?
```

### ✅ Goede Naamgevingsvoorbeelden

Juiste naamgeving drukt duidelijk de intentie en eigenschappen van status uit. Gebruik Observable (`$` suffix) of Signal en maak statustype (State, can, should) duidelijk.

```typescript
// Druk status duidelijk uit
readonly userLoadState$: Observable<'idle' | 'loading' | 'loaded' | 'error'>;

// Afgeleide status heeft ook duidelijke intentie
readonly canSubmit$: Observable<boolean>;
readonly shouldShowSpinner$: Observable<boolean>;

// Voorbeeld met Signal (bruikbaar in Angular, Preact, Solid.js, etc.)
readonly userLoadState = signal<LoadState>('idle');
readonly canSubmit = computed(() =>
  this.userLoadState() === 'loaded' && this.formValid()
);
```


## Diagnose Checklist

Controleer met de volgende checklist of je code niet in flag wildgroei probleem is gevallen. Gebruik als referentie bij code review of ontwerp.

```markdown
## 🚨 Gevaarsignalen

- [ ] 5 of meer boolean variabelen bestaan
- [ ] 3 of meer geneste `if`-statements binnen `subscribe`
- [ ] Dezelfde flag wordt op meerdere plaatsen ingesteld
- [ ] 3 of meer naamgevingen als `isXXXing`
- [ ] Statusbeheer laag bestaat maar component heeft status
- [ ] Meerdere naamgevingen als `xxxFlag`
- [ ] Foutafhandeling is verspreid over elke `subscribe`

## ✅ Tekenen van Verbetering

- [ ] Status wordt beheerd met `Observable` of `Signal`
- [ ] Afgeleide status is gedefinieerd met `map`/`computed`
- [ ] Statustransities zijn declaratief beschreven
- [ ] ViewModel patroon is toegepast
- [ ] Naamgeving drukt intentie duidelijk uit
```

## Samenvatting

Dit artikel legde de oorzaak en verbetermethoden uit van flag wildgroei problemen in RxJS projecten. Laten we ten slotte de belangrijke punten herhalen.

### Essentie van het Probleem

1. **17 flags hebben** ← Dit is een symptoom
2. **Ze zijn imperatieve mutable variabelen** ← Dit is de essentie
3. **Statustransities zijn niet declaratief** ← Dit is de oorzaak
4. **Naamgeving is vaag (xxxFlag)** ← Dit is de bron van verwarring

### Richting van Verbetering

Om het flag wildgroei probleem op te lossen, zijn de volgende 4 transformaties nodig.

- **boolean variabelen** → **Observable/Signal**
- **directe toewijzing** → **stream pipeline**
- **onafhankelijke 17** → **1 status + afgeleide status**
- **xxxFlag** → **xxxState$ / canXXX$**

### Het Belangrijkste

> [!IMPORTANT] Belangrijk Principe
> "Status is het resultaat van events, controleer niet direct met flags"

RxJS-introductie is niet een "syntax" transformatie maar een "filosofie" transformatie. Als je imperatief denken meetrekt, wordt flag hell niet opgelost. Door status als stream te beschouwen en declaratief te ontwerpen, verbeteren onderhoudbaarheid, leesbaarheid en testbaarheid allemaal.


## Gerelateerde Gedeelten

Om de kennis over flag beheer die je in dit artikel hebt geleerd verder te verdiepen, raadpleeg ook de volgende gerelateerde artikelen.

- [if-statement Nesting Hell in subscribe](./subscribe-if-hell) - Juiste verwerking van voorwaardelijke vertakkingen
- [Veelvoorkomende Fouten en Oplossingen](./common-mistakes) - Details van 15 anti-patronen
- [Foutafhandeling](/nl/guide/error-handling/strategies) - Juiste foutafhandelingsstrategieën
- [Subject en Multicast](/nl/guide/subjects/what-is-subject) - Basis van statusbeheer

## Referentiebronnen

Je kunt dieper leren met RxJS officiële documentatie en leerbronnen.

- [RxJS Officiële Documentatie](https://rxjs.dev/) - Officiële API referentie en gids
- [Learn RxJS](https://www.learnrxjs.io/) - Praktische voorbeelden per operator
- [RxJS Marbles](https://rxmarbles.com/) - Visueel begrip van operator gedrag
