---
title: Problème de prolifération de drapeaux de gestion d'état
description: "Explication de comment améliorer le problème de prolifération de 17 drapeaux booléens dans les projets RxJS avec une conception réactive. Présente des techniques de refactoring pratiques pour améliorer considérablement la maintenabilité et la lisibilité en exprimant l'état avec des Observables et en intégrant plusieurs drapeaux avec scan() et combineLatest()."
# outline: deep
---

# Prolifération de drapeaux de gestion d'état

Même dans les projets qui ont introduit RxJS, on voit souvent le problème de prolifération d'un grand nombre de drapeaux booléens dans les composants. Cet article explique les causes et les méthodes d'amélioration basées sur un cas réel où 17 drapeaux existaient.

## Exemple du problème

Examinons d'abord le code rencontré sur le terrain. Voici un exemple typique de prolifération de drapeaux de gestion d'état.

```typescript
class ProblematicComponent {
  // 17 drapeaux existent
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
    // Branchements complexes dans subscribe
    this.apiService.save(this.data).subscribe({
      next: (result) => {
        if (this.isLoading && !this.isSaving) {
          if (this.isFormValid && this.isDataLoaded) {
            if (!this.hasError && !this.isProcessing) {
              // Traitement réel
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

Un tel code se produit **même avec l'introduction de RxJS**. Ce pattern de gestion manuelle de 17 drapeaux et de contrôle par branchements conditionnels complexes pose des problèmes en termes de maintenabilité, lisibilité et testabilité.

## Pourquoi les drapeaux prolifèrent-ils ?

La prolifération de drapeaux est liée non seulement aux problèmes techniques, mais aussi aux patterns de pensée des développeurs et au processus d'évolution de l'organisation. Voici l'analyse de 5 causes principales.

### Analyse de la structure des causes

| Catégorie de cause | Symptômes concrets | Contexte |
|------------|------------|------|
| **① Persistance de pensée impérative** | Plus de 10 drapeaux comme `isLoading`, `isSaving`, `isError`<br>Nombreux gardes comme `if (this.isSaving) return;` | Contrôle par **"drapeaux d'état" impératifs** au lieu de flux RxJS.<br>État et effets de bord non séparés, faible lisibilité |
| **② Non-utilisation d'état dérivé** | Gestion par affectation directe côté composant comme `this.isLoaded = true;` | Bien que l'on puisse définir la dérivation d'état de manière déclarative en utilisant `map` ou `combineLatest` d'Observable,<br>composition manuelle de l'état sans cela |
| **③ Responsabilités de conception d'état ambiguës** | Plusieurs drapeaux pour le même état<br>(ex: `isLoadingStart`, `isLoadingEnd`) | **Traitement des changements d'état comme commandes**.<br>Dispersion de ce qui devrait être intégré comme "un état" en plusieurs drapeaux |
| **④ Branchements de flux RxJS non organisés** | Chaînes de multiples `if` et `tap` dans un `Observable`,<br>mélange d'effets de bord et de mises à jour d'état | Séparation des responsabilités de conception de flux non effectuée.<br>Utilisation ambiguë de `switchMap` et `catchError` |
| **⑤ Absence de couche ViewModel** | Manipulation directe de `this.isEditing`, `this.isSaved` dans le composant UI | En ayant l'état dans le composant,<br>les bénéfices de RxJS sont coupés |


## Cause fondamentale : incompatibilité des modèles de pensée

La cause fondamentale de la prolifération de drapeaux est **l'incompatibilité entre les modèles de pensée de programmation impérative et réactive**. Lorsque les développeurs utilisent RxJS avec une mentalité impérative, les problèmes suivants se produisent.

### Structure de la période de transition

De nombreux projets tombent dans l'enfer des drapeaux en suivant le processus d'évolution suivant.

```
1. Ajout de contrôle par drapeaux if pour faire fonctionner
   ↓
2. Introduction ultérieure de RxJS
   ↓
3. Coexistence sans pouvoir transformer l'ancienne logique en flux
   ↓
4. L'enfer des drapeaux est complet
```

### Mélange des couches de gestion d'état

L'état dans une application devrait être géré en 3 couches distinctes.

```
Application
 ├── État de vue (isOpen, isLoading, formDirty)     ← Dans le composant
 ├── État métier (entity, filters, errors)         ← Couche de gestion d'état
 └── État API (pending, success, error)            ← Flux RxJS
```

Si ces 3 couches ne sont pas séparées, même les "drapeaux" se mélangent en **3 types de responsabilités différentes**. Gérer l'état de vue et l'état API au même niveau augmente exponentiellement la complexité.

## Essence du problème : "nature" des drapeaux

Le vrai problème de la prolifération de drapeaux n'est pas "le nombre élevé" mais que **les drapeaux sont devenus des variables mutables impératives**. Comparons ci-dessous les drapeaux problématiques et les drapeaux appropriés.

### ❌ Drapeaux problématiques : variables mutables impératives

```typescript
class BadComponent {
  // Ceux-ci sont devenus des "commandes" au lieu d'"état"
  isLoading = false;
  isSaving = false;
  hasError = false;

  save() {
    if (this.isSaving) return;        // Clause de garde nécessaire
    this.isSaving = true;              // Changement manuel

    this.api.save().subscribe({
      next: () => {
        this.isSaving = false;         // Réinitialisation manuelle
        this.hasError = false;         // Gestion manuelle d'autres drapeaux
      },
      error: () => {
        this.isSaving = false;         // Même traitement à plusieurs endroits
        this.hasError = true;
      }
    });
  }
}
```

> [!WARNING] Problèmes
> - État "procédural" au lieu de "déclaratif"
> - Timing de changement d'état dispersé
> - Garantie manuelle de cohérence entre drapeaux par le développeur

### ✅ Drapeaux appropriés : variables réactives

```typescript
class GoodComponent {
  // Déclarer comme flux d'état
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
    shareReplay({ bufferSize: 1, refCount: true })
  );

  // Définir également l'état dérivé de manière déclarative
  readonly isLoading$ = this.saveState$.pipe(
    map(state => state === 'loading')
  );

  readonly hasError$ = this.saveState$.pipe(
    map(state => state === 'error')
  );

  save() {
    this.saveAction$.next(); // Seulement déclenchement d'événement
  }
}
```

> [!TIP] Améliorations
> - État géré centralement comme "flux"
> - Transition d'état définie de manière déclarative dans le pipeline
> - Cohérence entre drapeaux garantie automatiquement


## Critères de jugement de conception de drapeaux

Voici un résumé des critères pour juger si la conception de drapeaux de votre code est problématique. Utilisez-le comme référence lors de la revue de code ou de la conception.

| Aspect | ❌ Problématique | ✅ Pas de problème |
|------|-----------|-----------|
| **Type** | `boolean` (mutable) | `Observable<boolean>` / `Signal<boolean>` |
| **Méthode de changement** | Affectation directe `flag = true` | Flux/dérivation `state$.pipe(map(...))` |
| **Dépendances** | Implicites (ordre du code) | Explicites (combineLatest, computed) |
| **Nommage** | `xxxFlag`, `isXXX` (boolean) | `xxxState`, `canXXX`, `shouldXXX` |
| **Nombre** | 10+ booléens indépendants | 1 état + plusieurs dérivations |


## Stratégie d'amélioration

Pour résoudre le problème de prolifération de drapeaux, procédez par refactoring progressif en 3 étapes.

### Étape 1 : Inventaire de l'état

D'abord, énumérez tous les drapeaux actuels et classez-les par responsabilité. Cela permet de voir quels drapeaux peuvent être intégrés.

```typescript
// Énumérer les drapeaux existants et classer les responsabilités
interface StateInventory {
  view: string[];      // Contrôle d'affichage UI (isModalOpen, isEditing)
  business: string[];  // Logique métier (isFormValid, hasUnsavedChanges)
  api: string[];       // État de communication (isLoading, isSaving, hasError)
}
```

### Étape 2 : Conversion en Enum de l'état

Ensuite, intégrez plusieurs drapeaux booléens liés en un seul état. Par exemple, `isLoading`, `isSaving`, `hasError` peuvent tous être intégrés comme "état de requête".

```typescript
// Intégrer plusieurs booléens en un seul état
enum RequestState {
  Idle = 'idle',
  Loading = 'loading',
  Success = 'success',
  Error = 'error'
}

// Exemple d'utilisation
class Component {
  saveState: RequestState = RequestState.Idle;
  // isLoading, isSaving, hasError deviennent inutiles
}
```

### Étape 3 : Réactivisation

Enfin, gérez l'état avec Observable ou Signal, et définissez l'état dérivé de manière déclarative. Cela garantit automatiquement la cohérence de l'état.

```typescript
// Gérer avec Observable ou Signal
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

  // Intégrer comme ViewModel
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


## Exemple d'implémentation : refactoring de 17 drapeaux

Ici, nous montrons le processus de refactoring réel du composant avec 17 drapeaux présenté au début vers une conception réactive. Comparer Before/After permet de ressentir l'effet de l'amélioration.

### Before : gestion de drapeaux impérative

D'abord, revoyons le code problématique. 17 drapeaux booléens prolifèrent et sont contrôlés par des branchements conditionnels complexes.

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

### After : gestion d'état réactive

Ensuite, voyons le code amélioré. Les 17 drapeaux sont organisés en 3 états de base (apiState$, formState$, dataState$) et 1 état dérivé (vm$).

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
  // Gérer l'état de base avec Observable
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

  // Intégrer comme ViewModel (état dérivé)
  readonly vm$ = combineLatest([
    this.apiState$,
    this.formState$,
    this.dataState$,
    this.authService.isAuthenticated$
  ]).pipe(
    map(([api, form, data, auth]) => ({
      // État dérivé pour affichage UI
      canSave: !api.saving && form.valid && data.loaded && auth,
      showSpinner: api.loading || api.saving || api.deleting,
      showError: api.error !== null,
      errorMessage: api.error,
      // Exposer également l'état individuel si nécessaire
      isEditing: data.editing,
      formDirty: form.dirty
    }))
  );

  save() {
    // Le ViewModel vérifie automatiquement l'état
    this.apiState$.next({
      ...this.apiState$.value,
      saving: true,
      error: null
    });

    this.apiService.save().pipe(
      catchError((error: unknown) => {
        const message = error instanceof Error ? error.message : String(error);
        this.apiState$.next({
          ...this.apiState$.value,
          saving: false,
          error: message
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

### Utilisation côté UI

Grâce à la gestion d'état réactive, l'utilisation côté UI devient également beaucoup plus concise. Il n'est plus nécessaire de vérifier individuellement plusieurs drapeaux, il suffit d'obtenir les informations nécessaires du ViewModel.

```typescript
// Before : référence directe à plusieurs drapeaux
const isButtonDisabled =
  this.isLoading ||
  this.isSaving ||
  !this.isFormValid ||
  this.hasError ||
  !this.isDataLoaded;

// After : obtenir l'état dérivé du ViewModel
this.vm$.subscribe(vm => {
  const isButtonDisabled = !vm.canSave;
  const showSpinner = vm.showSpinner;
  const errorMessage = vm.errorMessage;
});
```


## Importance des conventions de nommage

Dans la conception de drapeaux, le nommage est très important. Un nommage approprié permet de comprendre en un coup d'œil la responsabilité, la nature et le cycle de vie du drapeau. À l'inverse, un nommage ambigu devient source de confusion.

### ❌ Mauvais exemples de nommage

Les nommages suivants ont une intention peu claire et réduisent la maintenabilité.

```typescript
// Quel drapeau ? Déclenché par quoi ?
userFlag: boolean;
dataFlag: boolean;
checkFlag: boolean;

// État ou action ?
isProcess: boolean;  // En cours de traitement ? Traité ?
```

### ✅ Bons exemples de nommage

Un nommage approprié exprime clairement l'intention et la nature de l'état. Utilisez Observable (suffixe `$`) ou Signal, et clarifiez le type d'état (State, can, should).

```typescript
// Exprimer clairement l'état
readonly userLoadState$: Observable<'idle' | 'loading' | 'loaded' | 'error'>;

// L'intention de l'état dérivé est également claire
readonly canSubmit$: Observable<boolean>;
readonly shouldShowSpinner$: Observable<boolean>;

// Exemple avec Signal (disponible dans Angular, Preact, Solid.js, etc.)
readonly userLoadState = signal<LoadState>('idle');
readonly canSubmit = computed(() =>
  this.userLoadState() === 'loaded' && this.formValid()
);
```


## Checklist de diagnostic

Vérifiez avec la checklist suivante si votre code est tombé dans le problème de prolifération de drapeaux. Utilisez-la comme référence lors de la revue de code ou de la conception.

```markdown
## 🚨 Signaux de danger

- [ ] 5 variables booléennes ou plus
- [ ] 3 imbrications ou plus d'instructions `if` dans `subscribe`
- [ ] Même drapeau défini à plusieurs endroits
- [ ] 3 nommages `isXXXing` ou plus
- [ ] État dans le composant malgré une couche de gestion d'état
- [ ] Plusieurs nommages `xxxFlag`
- [ ] Gestion d'erreurs dispersée dans chaque `subscribe`

## ✅ Signes d'amélioration

- [ ] État géré avec `Observable` ou `Signal`
- [ ] État dérivé défini avec `map`/`computed`
- [ ] Transition d'état écrite de manière déclarative
- [ ] Pattern ViewModel appliqué
- [ ] Nommage exprimant clairement l'intention
```

## Résumé

Cet article a expliqué les causes et les méthodes d'amélioration du problème de prolifération de drapeaux dans les projets RxJS. Revoyons les points importants.

### Essence du problème

1. **Avoir 17 drapeaux** ← C'est le symptôme
2. **Qu'ils soient des variables mutables impératives** ← C'est l'essence
3. **Que la transition d'état ne soit pas déclarative** ← C'est la cause
4. **Que le nommage soit ambigu (xxxFlag)** ← C'est source de confusion

### Direction d'amélioration

Pour résoudre le problème de prolifération de drapeaux, 4 conversions sont nécessaires.

- **Variables booléennes** → **Observable/Signal**
- **Affectation directe** → **Pipeline de flux**
- **17 indépendants** → **1 état + états dérivés**
- **xxxFlag** → **xxxState$ / canXXX$**

### Le plus important

> [!IMPORTANT] Principe important
> "L'état est le résultat d'événements, ne pas le contrôler directement avec des drapeaux"

L'introduction de RxJS est une conversion de "pensée" et non de "syntaxe". Si vous traînez la pensée impérative, l'enfer des drapeaux ne sera pas résolu. En concevant l'état comme un flux de manière déclarative, maintenabilité, lisibilité et testabilité s'améliorent toutes.


## Sections connexes

Pour approfondir les connaissances sur la gestion de drapeaux apprises dans cet article, consultez également les articles connexes suivants.

- [Enfer d'imbrication if dans subscribe](./subscribe-if-hell) - Méthode appropriée de traitement des branchements conditionnels
- [Erreurs courantes et solutions](./common-mistakes) - Détails de 15 anti-patterns
- [Gestion des erreurs](/fr/guide/error-handling/strategies) - Stratégies appropriées de gestion d'erreurs
- [Subjects et multicasting](/fr/guide/subjects/what-is-subject) - Bases de la gestion d'état

## Ressources de référence

Approfondissez avec la documentation officielle RxJS et les ressources d'apprentissage.

- [Documentation officielle RxJS](https://rxjs.dev/) - Référence API officielle et guide
- [Learn RxJS](https://www.learnrxjs.io/) - Exemples pratiques par opérateur
- [RxJS Marbles](https://rxmarbles.com/) - Comprendre visuellement le comportement des opérateurs
