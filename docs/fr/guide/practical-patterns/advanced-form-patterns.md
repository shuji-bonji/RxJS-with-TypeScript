---
description: "Modèles avancés de formulaires utilisant JSON Patch. Apprenez à implémenter la sauvegarde automatique et Undo/Redo pour les formulaires à grande échelle, la synchronisation temps réel pour l'édition collaborative, la gestion hors ligne, le suivi de l'historique des opérations et plus encore avec RxJS et TypeScript au niveau entreprise."
---

# Modèles avancés de formulaires avec JSON Patch

Lors de l'implémentation de formulaires à grande échelle ou d'édition collaborative en temps réel, l'approche traditionnelle consistant à "soumettre le formulaire entier" pose des défis en termes de performances et d'expérience utilisateur.

Cet article explique les modèles avancés de formulaires utilisant **JSON Patch (RFC 6902)**. En envoyant uniquement les différences, vous pouvez réduire la bande passante réseau et implémenter efficacement Undo/Redo et l'édition collaborative.

## Ce que vous apprendrez dans cet article

- Fondamentaux de JSON Patch/Pointer (RFC 6902/6901)
- Sauvegarde automatique pour formulaires à grande échelle (basée sur les différences)
- Implémentation d'Undo/Redo (patches inverses)
- Synchronisation temps réel pour édition collaborative
- Bases d'Operational Transform (OT) / CRDT
- Modèles d'intégration WebSocket et RxJS
- Résolution de conflits et gestion de versions

> [!TIP] Connaissances préalables
> Cet article suppose une connaissance de [Chapitre 4 : Opérateurs](../operators/index.md), [Modèles de traitement de formulaires](./form-handling.md) et [Traitement de données temps réel](./real-time-data.md).

> [!NOTE] Quand ce modèle est-il nécessaire
> - **Formulaires à grande échelle** (100+ champs) nécessitant une sauvegarde automatique
> - Fonctionnalité **Undo/Redo** indispensable
> - **Édition collaborative temps réel** (fonctionnalité type Google Docs)
> - **Gestion hors ligne** nécessitant la mise en file d'attente des différences
>
> Pour les petits formulaires (~20 champs), [les modèles de traitement de formulaires normaux](./form-handling.md) sont suffisants.

## Fondamentaux de JSON Patch/Pointer

### Qu'est-ce que JSON Patch

**JSON Patch (RFC 6902)** est un format standard pour représenter les modifications d'un document JSON. Vous pouvez envoyer **uniquement les modifications** au lieu du formulaire entier.

```typescript
// Données du formulaire avant modification
const before = {
  profile: {
    name: "Tanaka Taro",
    email: "tanaka@example.com",
    age: 30
  }
};

// Données du formulaire après modification
const after = {
  profile: {
    name: "Tanaka Taro",
    email: "tanaka.updated@example.com", // Modifié
    age: 31 // Modifié
  }
};

// JSON Patch (différence)
const patch = [
  { op: "replace", path: "/profile/email", value: "tanaka.updated@example.com" },
  { op: "replace", path: "/profile/age", value: 31 }
];
```

> [!NOTE] Les 6 opérations de JSON Patch
> - `add` - Ajouter une valeur
> - `remove` - Supprimer une valeur
> - `replace` - Remplacer une valeur
> - `move` - Déplacer une valeur
> - `copy` - Copier une valeur
> - `test` - Tester une valeur (validation)

### Qu'est-ce que JSON Pointer

**JSON Pointer (RFC 6901)** est une notation de chemin pour pointer vers une valeur spécifique dans un document JSON.

```typescript
const formData = {
  user: {
    profile: {
      name: "Tanaka Taro"
    },
    settings: {
      notifications: true
    }
  }
};

// Exemples de JSON Pointer
"/user/profile/name"           // → "Tanaka Taro"
"/user/settings/notifications" // → true
"/user/profile"                // → { name: "Tanaka Taro" }
```

### Détection de différences avec RxJS

Combinez `pairwise()` et la bibliothèque `fast-json-patch` pour détecter automatiquement les modifications du formulaire.

```typescript
import { BehaviorSubject, pairwise, map } from 'rxjs';
import { compare } from 'fast-json-patch';

interface FormData {
  profile: {
    name: string;
    email: string;
    age: number;
  };
}

const initialData: FormData = {
  profile: {
    name: "",
    email: "",
    age: 0
  }
};

const formData$ = new BehaviorSubject<FormData>(initialData);

// Détecter les différences
const patches$ = formData$.pipe(
  pairwise(), // Obtenir la paire [valeur précédente, valeur actuelle]
  map(([previous, current]) => compare(previous, current))
);

patches$.subscribe(patches => {
  console.log('Modifications détectées:', patches);
  // Exemple: [{ op: "replace", path: "/profile/name", value: "Tanaka Taro" }]
});

// Simuler une mise à jour de formulaire
formData$.next({
  profile: {
    name: "Tanaka Taro",
    email: "tanaka@example.com",
    age: 30
  }
});
```

> [!TIP] Bibliothèque fast-json-patch
> ```bash
> npm install fast-json-patch
> ```
> - Conforme RFC 6902
> - Génération (`compare`) et application (`applyPatch`) de différences
> - Génération de patches inverses (pour Undo)
> - Support TypeScript

---

## Sauvegarde automatique et Undo/Redo pour formulaires à grande échelle

Implémentez la sauvegarde automatique et la fonctionnalité Undo/Redo pour les formulaires à grande échelle (par exemple : inscription membre avec 100 champs, écran de gestion de produits).

### Stratégie d'implémentation

**Responsabilités du frontend :**
- Génération et alignement des opérations (modifications)
- Réflexion UI optimiste (application immédiate avec `scan`)
- Gestion de la pile Undo/Redo (historique avec patches inverses)
- Gestion de la file d'envoi (garantie d'ordre avec `concatMap`)
- Batching (`bufferTime` + compression)

**Responsabilités du backend :**
- Gestion de versions (Vector Clock / timestamp)
- Garantie d'idempotence (détection de doublons par Request ID)
- Persistance et logs d'audit

### Modèle 1 : Sauvegarde automatique de base

Détecte les modifications du formulaire et les envoie par lots au serveur à intervalles réguliers.

```typescript
import {
  BehaviorSubject,
  Subject,
  pairwise,
  map,
  bufferTime,
  filter,
  concatMap,
  catchError,
  of
} from 'rxjs';
import { compare, Operation } from 'fast-json-patch';

interface LargeFormData {
  personalInfo: {
    firstName: string;
    lastName: string;
    email: string;
    phone: string;
    dateOfBirth: string;
  };
  address: {
    street: string;
    city: string;
    state: string;
    zipCode: string;
    country: string;
  };
  preferences: {
    newsletter: boolean;
    notifications: boolean;
    theme: 'light' | 'dark';
  };
  // ... Suppose plus de 100 champs
}

const initialFormData: LargeFormData = {
  personalInfo: {
    firstName: "",
    lastName: "",
    email: "",
    phone: "",
    dateOfBirth: ""
  },
  address: {
    street: "",
    city: "",
    state: "",
    zipCode: "",
    country: "Japan"
  },
  preferences: {
    newsletter: false,
    notifications: true,
    theme: 'light'
  }
};

// Flux de données du formulaire
const formData$ = new BehaviorSubject<LargeFormData>(initialFormData);

// Flux de résultats de sauvegarde
const saveResult$ = new Subject<{ success: boolean; message: string }>();

// Élément d'affichage de statut pour démo
const statusDiv = document.createElement('div');
statusDiv.style.padding = '10px';
statusDiv.style.margin = '10px';
statusDiv.style.border = '2px solid #ccc';
statusDiv.style.borderRadius = '4px';
statusDiv.style.fontFamily = 'monospace';
statusDiv.style.fontSize = '14px';
document.body.appendChild(statusDiv);

function updateStatus(message: string, color: string = '#333') {
  statusDiv.innerHTML = `<span style="color: ${color}">${message}</span>`;
}

// Pipeline de sauvegarde automatique
formData$.pipe(
  pairwise(),
  map(([previous, current]) => ({
    patches: compare(previous, current),
    timestamp: Date.now()
  })),
  filter(({ patches }) => patches.length > 0), // Ignorer si pas de modifications
  bufferTime(2000), // Buffer des modifications sur 2 secondes
  filter(buffer => buffer.length > 0), // Ignorer les buffers vides
  map(buffer => {
    // Consolider tous les patches du buffer en un seul tableau
    const allPatches = buffer.flatMap(item => item.patches);
    updateStatus(`📦 Traitement par lot de ${allPatches.length} modifications...`, '#FF9800');
    return allPatches;
  }),
  concatMap(patches => saveToServer(patches)), // Envoyer avec garantie d'ordre
  catchError(error => {
    console.error('Erreur de sauvegarde automatique:', error);
    updateStatus(`❌ Échec de sauvegarde: ${error.message}`, '#f44336');
    return of({ success: false, message: error.message });
  })
).subscribe(result => {
  if (result.success) {
    updateStatus(`✅ Sauvegarde automatique terminée (${new Date().toLocaleTimeString()})`, '#4CAF50');
  }
  saveResult$.next(result);
});

// Sauvegarde sur le serveur (implémentation mock)
function saveToServer(patches: Operation[]): Promise<{ success: boolean; message: string }> {
  console.log('Envoi au serveur:', patches);

  // Exemple d'implémentation réelle :
  // return fetch('/api/forms/12345/patches', {
  //   method: 'PATCH',
  //   headers: { 'Content-Type': 'application/json-patch+json' },
  //   body: JSON.stringify(patches)
  // }).then(res => res.json());

  // Mock : retourne succès après 500ms
  return new Promise(resolve => {
    setTimeout(() => {
      resolve({
        success: true,
        message: `${patches.length} modifications sauvegardées`
      });
    }, 500);
  });
}

// Démo : Simuler une modification de formulaire
const demoButton = document.createElement('button');
demoButton.textContent = 'Modifier le formulaire (Démo)';
demoButton.style.padding = '10px 20px';
demoButton.style.margin = '10px';
demoButton.style.fontSize = '16px';
demoButton.style.cursor = 'pointer';
document.body.appendChild(demoButton);

demoButton.addEventListener('click', () => {
  // Modifier aléatoirement des champs
  const currentData = formData$.getValue();
  const updatedData = {
    ...currentData,
    personalInfo: {
      ...currentData.personalInfo,
      firstName: `Taro_${Math.floor(Math.random() * 100)}`,
      email: `taro${Math.floor(Math.random() * 100)}@example.com`
    },
    preferences: {
      ...currentData.preferences,
      newsletter: !currentData.preferences.newsletter
    }
  };
  formData$.next(updatedData);
  updateStatus('📝 Formulaire modifié...', '#2196F3');
});
```

> [!NOTE] Points de la sauvegarde automatique
> - **`bufferTime(2000)`** - Regrouper les modifications sur 2 secondes pour l'efficacité réseau
> - **`concatMap`** - Garantir l'ordre des patches (`mergeMap` peut briser l'ordre)
> - **`filter`** - Ignorer si pas de modifications (réduire les requêtes inutiles)
> - **Idempotence** - Sécurisé même si le même patch est envoyé plusieurs fois (ajouter Request ID)

### Modèle 2 : Implémentation Undo/Redo

Utilisez les patches inverses pour implémenter la fonctionnalité Undo/Redo.

```typescript
import { Subject, scan } from 'rxjs';
import { applyPatch, Operation, deepClone } from 'fast-json-patch';

interface HistoryState<T> {
  current: T;
  undoStack: Operation[][];
  redoStack: Operation[][];
}

interface HistoryAction {
  type: 'APPLY_PATCH' | 'UNDO' | 'REDO';
  patches?: Operation[];
}

// Flux de gestion de l'historique
const historyAction$ = new Subject<HistoryAction>();

const initialState: HistoryState<LargeFormData> = {
  current: initialFormData,
  undoStack: [],
  redoStack: []
};

// Reducer pour gérer l'historique
const history$ = historyAction$.pipe(
  scan((state, action) => {
    switch (action.type) {
      case 'APPLY_PATCH':
        if (!action.patches || action.patches.length === 0) return state;

        // Appliquer le patch
        const cloned = deepClone(state.current);
        const result = applyPatch(cloned, action.patches, true, false);

        return {
          current: result.newDocument,
          undoStack: [...state.undoStack, action.patches],
          redoStack: [] // Effacer la pile Redo avec nouvelle opération
        };

      case 'UNDO':
        if (state.undoStack.length === 0) return state;

        const patchesToUndo = state.undoStack[state.undoStack.length - 1];
        const beforeUndo = deepClone(state.current);

        // Générer et appliquer le patch inverse
        const inversePatch = generateInversePatch(state.current, patchesToUndo);
        const undoResult = applyPatch(beforeUndo, inversePatch, true, false);

        return {
          current: undoResult.newDocument,
          undoStack: state.undoStack.slice(0, -1),
          redoStack: [...state.redoStack, patchesToUndo]
        };

      case 'REDO':
        if (state.redoStack.length === 0) return state;

        const patchesToRedo = state.redoStack[state.redoStack.length - 1];
        const beforeRedo = deepClone(state.current);
        const redoResult = applyPatch(beforeRedo, patchesToRedo, true, false);

        return {
          current: redoResult.newDocument,
          undoStack: [...state.undoStack, patchesToRedo],
          redoStack: state.redoStack.slice(0, -1)
        };

      default:
        return state;
    }
  }, initialState)
);

// Génération de patch inverse (implémentation simplifiée)
function generateInversePatch(document: any, patches: Operation[]): Operation[] {
  // applyPatch de fast-json-patch retourne le patch inverse si le 4ème argument est true
  const cloned = deepClone(document);
  const result = applyPatch(cloned, patches, true, true);
  return result[1] || []; // Obtenir le patch inverse
}

// Éléments UI
const historyControlDiv = document.createElement('div');
historyControlDiv.style.padding = '10px';
historyControlDiv.style.margin = '10px';
document.body.appendChild(historyControlDiv);

const undoButton = document.createElement('button');
undoButton.textContent = '↶ Undo';
undoButton.style.padding = '10px 20px';
undoButton.style.marginRight = '10px';
undoButton.style.fontSize = '16px';
undoButton.style.cursor = 'pointer';
historyControlDiv.appendChild(undoButton);

const redoButton = document.createElement('button');
redoButton.textContent = '↷ Redo';
redoButton.style.padding = '10px 20px';
redoButton.style.fontSize = '16px';
redoButton.style.cursor = 'pointer';
historyControlDiv.appendChild(redoButton);

const historyInfo = document.createElement('div');
historyInfo.style.marginTop = '10px';
historyInfo.style.fontFamily = 'monospace';
historyInfo.style.fontSize = '14px';
historyControlDiv.appendChild(historyInfo);

// Afficher l'état de l'historique
history$.subscribe(state => {
  undoButton.disabled = state.undoStack.length === 0;
  redoButton.disabled = state.redoStack.length === 0;

  historyInfo.innerHTML = `
    📚 Undo possible: ${state.undoStack.length} fois<br>
    📚 Redo possible: ${state.redoStack.length} fois<br>
    📝 Valeur actuelle: ${JSON.stringify(state.current.personalInfo.firstName)}
  `;

  // Synchroniser les données du formulaire
  formData$.next(state.current);
});

// Événements des boutons
undoButton.addEventListener('click', () => {
  historyAction$.next({ type: 'UNDO' });
});

redoButton.addEventListener('click', () => {
  historyAction$.next({ type: 'REDO' });
});

// Démo : Bouton d'application de patch
const applyPatchButton = document.createElement('button');
applyPatchButton.textContent = 'Appliquer modification (Test Undo/Redo)';
applyPatchButton.style.padding = '10px 20px';
applyPatchButton.style.margin = '10px';
applyPatchButton.style.fontSize = '16px';
applyPatchButton.style.cursor = 'pointer';
document.body.appendChild(applyPatchButton);

applyPatchButton.addEventListener('click', () => {
  const patches: Operation[] = [
    { op: 'replace', path: '/personalInfo/firstName', value: `Taro_${Date.now()}` }
  ];
  historyAction$.next({ type: 'APPLY_PATCH', patches });
});
```

> [!TIP] Points d'Undo/Redo
> - **Patch inverse** - Le 4ème argument de `applyPatch` à `true` permet d'obtenir le patch inverse
> - **Gestion de pile** - Pile Undo (opérations passées) et pile Redo (opérations annulées)
> - **Effacement Redo lors de nouvelle opération** - Réinitialiser la pile Redo lors d'une modification
> - **Modèle Reducer avec `scan`** - Implémentation de la gestion d'état comme useReducer de React

### Modèle 3 : Gestion hors ligne (File IndexedDB)

Mettre en file d'attente les modifications dans IndexedDB hors ligne et synchroniser lors du retour en ligne.

```typescript
import { fromEvent, merge, map, filter, concatMap, catchError, of } from 'rxjs';

// État en ligne/hors ligne
const online$ = merge(
  fromEvent(window, 'online').pipe(map(() => true)),
  fromEvent(window, 'offline').pipe(map(() => false))
);

// Opérations IndexedDB (implémentation simplifiée)
class PatchQueue {
  private dbName = 'form-patches';
  private storeName = 'patches';
  private db: IDBDatabase | null = null;

  async init(): Promise<void> {
    return new Promise((resolve, reject) => {
      const request = indexedDB.open(this.dbName, 1);

      request.onerror = () => reject(request.error);
      request.onsuccess = () => {
        this.db = request.result;
        resolve();
      };

      request.onupgradeneeded = (event) => {
        const db = (event.target as IDBOpenDBRequest).result;
        if (!db.objectStoreNames.contains(this.storeName)) {
          db.createObjectStore(this.storeName, {
            keyPath: 'id',
            autoIncrement: true
          });
        }
      };
    });
  }

  async enqueue(patches: Operation[]): Promise<void> {
    if (!this.db) throw new Error('DB not initialized');

    return new Promise((resolve, reject) => {
      const transaction = this.db!.transaction([this.storeName], 'readwrite');
      const store = transaction.objectStore(this.storeName);
      const request = store.add({
        patches,
        timestamp: Date.now()
      });

      request.onsuccess = () => resolve();
      request.onerror = () => reject(request.error);
    });
  }

  async dequeueAll(): Promise<Operation[][]> {
    if (!this.db) throw new Error('DB not initialized');

    return new Promise((resolve, reject) => {
      const transaction = this.db!.transaction([this.storeName], 'readwrite');
      const store = transaction.objectStore(this.storeName);
      const request = store.getAll();

      request.onsuccess = () => {
        const items = request.result;
        // Effacer après récupération
        store.clear();
        resolve(items.map((item: any) => item.patches));
      };
      request.onerror = () => reject(request.error);
    });
  }
}

const patchQueue = new PatchQueue();
patchQueue.init().then(() => {
  console.log('IndexedDB initialized');
});

// Sauvegarde automatique avec gestion hors ligne
formData$.pipe(
  pairwise(),
  map(([previous, current]) => compare(previous, current)),
  filter(patches => patches.length > 0),
  bufferTime(2000),
  filter(buffer => buffer.length > 0),
  map(buffer => buffer.flatMap(patches => patches)),
  concatMap(async (patches) => {
    const isOnline = navigator.onLine;

    if (isOnline) {
      // En ligne : Envoyer au serveur
      try {
        return await saveToServer(patches);
      } catch (error) {
        // Échec d'envoi : Ajouter à la file
        await patchQueue.enqueue(patches);
        return { success: false, message: 'Ajouté à la file hors ligne' };
      }
    } else {
      // Hors ligne : Ajouter à la file
      await patchQueue.enqueue(patches);
      console.log('📴 Hors ligne : Ajouté à la file');
      return { success: false, message: 'Hors ligne' };
    }
  })
).subscribe();

// Synchroniser lors du retour en ligne
online$.pipe(
  filter(isOnline => isOnline),
  concatMap(async () => {
    console.log('🌐 Retour en ligne : Synchronisation de la file...');
    const queuedPatches = await patchQueue.dequeueAll();

    for (const patches of queuedPatches) {
      await saveToServer(patches);
    }

    return { synced: queuedPatches.length };
  })
).subscribe(result => {
  console.log(`✅ ${result.synced} patches synchronisés`);
});
```

> [!NOTE] Points de la gestion hors ligne
> - **IndexedDB** - Stockage persistant côté navigateur (plus grande capacité que LocalStorage)
> - **File d'attente** - Accumuler dans la file d'envoi hors ligne
> - **Détection retour en ligne** - Surveiller avec `window.addEventListener('online')`
> - **Garantie d'ordre** - Envoyer les patches de la file dans l'ordre avec `concatMap`

---

## Synchronisation temps réel pour édition collaborative

Implémentez l'édition collaborative temps réel où plusieurs utilisateurs éditent le même formulaire simultanément (fonctionnalité type Google Docs, Notion, Figma).

### Fondamentaux d'Operational Transform (OT) et CRDT

Dans l'édition collaborative temps réel, **la résolution de conflits** est le plus grand défi. Il existe deux approches principales.

#### Operational Transform (OT)

Résout les conflits en transformant les opérations de modification.

```
Utilisateur A : "hello" → "hello world" (ajout " world" à la fin)
Utilisateur B : "hello" → "Hi hello"   (ajout "Hi " au début)

【Sans transformation】
Résultat : "Hi hello world" ou "hello world" (dernier gagne)

【Avec OT】
- Transformer l'opération de l'utilisateur A par celle de B
- Transformer l'opération de l'utilisateur B par celle de A
→ Résultat : "Hi hello world" (conserve les deux modifications)
```

**Avantages :**
- Résultat intuitif (conserve les deux modifications)
- Le serveur décide de l'état final

**Inconvénients :**
- Implémentation complexe
- Serveur obligatoire

#### CRDT (Conflict-free Replicated Data Types)

Utilise des structures de données mathématiquement sans conflit.

```
Attribuer un ID unique à chaque caractère :

Utilisateur A : [h1, e2, l3, l4, o5] → [h1, e2, l3, l4, o5, space6, w7, o8, r9, l10, d11]
Utilisateur B : [h1, e2, l3, l4, o5] → [H12, i13, space14, h1, e2, l3, l4, o5]

Lors du merge, trier par ID :
→ [H12, i13, space14, h1, e2, l3, l4, o5, space6, w7, o8, r9, l10, d11]
→ "Hi hello world"
```

**Avantages :**
- Pas de serveur nécessaire (P2P possible)
- Gestion hors ligne facile

**Inconvénients :**
- Utilisation mémoire importante
- Traitement de suppression complexe (méthode Tombstone)

> [!TIP] Choix de bibliothèque
> - **Implémentation OT** : [ShareDB](https://github.com/share/sharedb) - Operational Transform
> - **Implémentation CRDT** : [Yjs](https://github.com/yjs/yjs) - CRDT haute performance (recommandé)
> - **Implémentation CRDT** : [Automerge](https://github.com/automerge/automerge) - Spécialisé JSON

Cet article présente un exemple d'implémentation avec **Yjs (CRDT)**.

### Modèle 4 : Édition collaborative avec Yjs

Combinez Yjs et RxJS pour implémenter l'édition collaborative temps réel.

```typescript
import { fromEvent, merge, Subject } from 'rxjs';
import * as Y from 'yjs';
import { WebsocketProvider } from 'y-websocket';

// Document Yjs (état partagé)
const ydoc = new Y.Doc();

// Données de formulaire partagées (type Map Yjs)
const yFormData = ydoc.getMap('formData');

// Provider WebSocket (connexion serveur)
// En production, construire votre propre serveur WebSocket
const wsProvider = new WebsocketProvider(
  'wss://demos.yjs.dev', // Serveur public pour démo
  'rxjs-form-demo',      // Nom de salle
  ydoc
);

// Subject RxJS pour les modifications de formulaire
const formChange$ = new Subject<{
  key: string;
  value: any;
  user: string;
}>();

// Convertir les modifications Yjs en flux RxJS
yFormData.observe(event => {
  event.changes.keys.forEach((change, key) => {
    if (change.action === 'add' || change.action === 'update') {
      formChange$.next({
        key: key,
        value: yFormData.get(key),
        user: 'remote'
      });
    }
  });
});

// UI de démo
const collaborativeFormDiv = document.createElement('div');
collaborativeFormDiv.style.padding = '20px';
collaborativeFormDiv.style.margin = '10px';
collaborativeFormDiv.style.border = '2px solid #2196F3';
collaborativeFormDiv.style.borderRadius = '8px';
collaborativeFormDiv.style.backgroundColor = '#f5f5f5';
document.body.appendChild(collaborativeFormDiv);

const title = document.createElement('h3');
title.textContent = '🤝 Démo d\'édition collaborative temps réel';
title.style.margin = '0 0 15px 0';
collaborativeFormDiv.appendChild(title);

const nameInput = document.createElement('input');
nameInput.type = 'text';
nameInput.placeholder = 'Entrez votre nom (synchronisé avec autres utilisateurs)';
nameInput.style.padding = '10px';
nameInput.style.width = '100%';
nameInput.style.fontSize = '16px';
nameInput.style.border = '2px solid #ccc';
nameInput.style.borderRadius = '4px';
nameInput.style.boxSizing = 'border-box';
collaborativeFormDiv.appendChild(nameInput);

const syncStatus = document.createElement('div');
syncStatus.style.marginTop = '10px';
syncStatus.style.fontSize = '14px';
syncStatus.style.color = '#666';
collaborativeFormDiv.appendChild(syncStatus);

const activeUsers = document.createElement('div');
activeUsers.style.marginTop = '10px';
activeUsers.style.fontSize = '14px';
activeUsers.style.color = '#666';
collaborativeFormDiv.appendChild(activeUsers);

// Surveiller l'état de connexion
wsProvider.on('status', (event: { status: string }) => {
  if (event.status === 'connected') {
    syncStatus.innerHTML = '🟢 <strong>Connecté</strong> - Synchronisation temps réel avec autres utilisateurs';
    syncStatus.style.color = '#4CAF50';
  } else {
    syncStatus.innerHTML = '🔴 <strong>Déconnecté</strong>';
    syncStatus.style.color = '#f44336';
  }
});

// Afficher le nombre d'utilisateurs connectés (API Awareness)
wsProvider.awareness.on('change', () => {
  const users = Array.from(wsProvider.awareness.getStates().keys());
  activeUsers.innerHTML = `👥 Utilisateurs actifs : ${users.length}`;
});

// Refléter les modifications locales dans Yjs
let isRemoteChange = false;

fromEvent(nameInput, 'input').subscribe(() => {
  if (!isRemoteChange) {
    yFormData.set('name', nameInput.value);
  }
});

// Refléter les modifications distantes dans l'UI
formChange$.subscribe(change => {
  if (change.key === 'name') {
    isRemoteChange = true;
    nameInput.value = change.value || '';
    nameInput.style.borderColor = '#4CAF50';

    setTimeout(() => {
      nameInput.style.borderColor = '#ccc';
      isRemoteChange = false;
    }, 500);
  }
});

// Définir la valeur initiale
const initialName = yFormData.get('name');
if (initialName) {
  nameInput.value = initialName;
}
```

> [!NOTE] Points de Yjs
> - **Y.Doc** - Document partagé (CRDT)
> - **Y.Map** - Type Map partagé (`{ key: value }`)
> - **WebsocketProvider** - Synchronisation via WebSocket
> - **API Awareness** - Informations de présence (utilisateurs connectés, position du curseur)
> - **observe** - Surveiller les modifications et convertir en flux RxJS

### Modèle 5 : Gestion de présence (Partage de position du curseur)

Visualisez qui édite où.

```typescript
import { throttleTime } from 'rxjs';

interface UserPresence {
  userId: string;
  name: string;
  color: string;
  cursorPosition: number;
  timestamp: number;
}

// Générer une couleur aléatoire
function generateRandomColor(): string {
  const colors = ['#f44336', '#E91E63', '#9C27B0', '#673AB7', '#3F51B5', '#2196F3', '#00BCD4', '#009688'];
  return colors[Math.floor(Math.random() * colors.length)];
}

// ID utilisateur personnel (généré aléatoirement)
const myUserId = `user_${Math.random().toString(36).substr(2, 9)}`;
const myColor = generateRandomColor();

// Configuration Awareness (informations de présence)
wsProvider.awareness.setLocalState({
  userId: myUserId,
  name: `Utilisateur${myUserId.slice(-4)}`,
  color: myColor
});

// Détecter les modifications de position du curseur (limiter la fréquence d'envoi avec throttle)
fromEvent(nameInput, 'selectionchange').pipe(
  throttleTime(200)
).subscribe(() => {
  const cursorPosition = nameInput.selectionStart || 0;

  wsProvider.awareness.setLocalStateField('cursorPosition', cursorPosition);
});

// Afficher la position du curseur des autres utilisateurs
const cursorOverlay = document.createElement('div');
cursorOverlay.style.position = 'relative';
cursorOverlay.style.marginTop = '10px';
cursorOverlay.style.fontSize = '12px';
collaborativeFormDiv.appendChild(cursorOverlay);

wsProvider.awareness.on('change', () => {
  const states = wsProvider.awareness.getStates();
  const cursors: string[] = [];

  states.forEach((state: any, clientId: number) => {
    if (state.userId !== myUserId) {
      cursors.push(
        `<span style="color: ${state.color}">● ${state.name}</span> (position : ${state.cursorPosition || 0})`
      );
    }
  });

  cursorOverlay.innerHTML = cursors.length > 0
    ? `📍 Autres utilisateurs : ${cursors.join(', ')}`
    : '📍 Aucun autre utilisateur';
});
```

> [!TIP] Points de gestion de présence
> - **`awareness.setLocalState`** - Partager ses propres informations
> - **`awareness.getStates`** - Obtenir les informations de tous les utilisateurs
> - **`throttleTime`** - Limiter la fréquence d'envoi des déplacements du curseur (200ms)
> - **Code couleur** - Attribuer une couleur à chaque utilisateur pour améliorer la visibilité

### Modèle 6 : Gestion d'erreurs et reconnexion

Implémentez la reconnexion et la gestion d'erreurs lors de la déconnexion WebSocket.

```typescript
import { timer, takeUntil, Subject } from 'rxjs';

const disconnect$ = new Subject<void>();

// Détection de déconnexion WebSocket
wsProvider.on('connection-close', () => {
  console.warn('⚠️ Déconnexion WebSocket');
  syncStatus.innerHTML = '🟡 <strong>Reconnexion...</strong>';
  syncStatus.style.color = '#FF9800';

  // Tenter de reconnecter après 5 secondes
  timer(5000).pipe(
    takeUntil(disconnect$)
  ).subscribe(() => {
    console.log('🔄 Tentative de reconnexion');
    wsProvider.connect();
  });
});

// Traitement d'erreur WebSocket
wsProvider.on('connection-error', (error: Error) => {
  console.error('❌ Erreur WebSocket:', error);
  syncStatus.innerHTML = `❌ <strong>Erreur:</strong> ${error.message}`;
  syncStatus.style.color = '#f44336';
});

// Nettoyage
window.addEventListener('beforeunload', () => {
  disconnect$.next();
  wsProvider.disconnect();
  ydoc.destroy();
});
```

> [!WARNING] Points d'attention en production
> - **Serveur WebSocket personnalisé** - `wss://demos.yjs.dev` est pour développement. En production, construire [y-websocket-server](https://github.com/yjs/y-websocket)
> - **Authentification** - Implémenter l'authentification par token lors de la connexion WebSocket
> - **Scaling** - Partager l'état entre serveurs WebSocket via Redis etc.
> - **Persistance** - Sauvegarder les documents Yjs en base de données (`y-leveldb`, `y-indexeddb`)

---

## Résumé

Cet article a expliqué les modèles avancés de formulaires utilisant JSON Patch.

### Points importants

> [!IMPORTANT] Critères de sélection des modèles JSON Patch
>
> **Quand le traitement normal de formulaire est suffisant :**
> - Nombre de champs : ~20
> - Sauvegarde automatique : Non nécessaire ou envoi complet OK
> - Undo/Redo : Non nécessaire
> - Édition collaborative : Non nécessaire
> → Utiliser [les modèles de traitement de formulaires normaux](./form-handling.md)
>
> **Quand JSON Patch est nécessaire :**
> - Nombre de champs : 100+
> - Sauvegarde automatique : Indispensable (envoi différences uniquement)
> - Undo/Redo : Indispensable
> - Édition collaborative : Synchronisation temps réel nécessaire
> → Utiliser les modèles de cet article

### Résumé des modèles d'implémentation

| Modèle | Cas d'usage | Technologies principales |
|---------|------------|---------|
| **Sauvegarde automatique de base** | Envoi de différences pour formulaires à grande échelle | `pairwise` + `bufferTime` + `concatMap` |
| **Undo/Redo** | Gestion de l'historique des opérations | Patches inverses + `scan` |
| **Gestion hors ligne** | Gestion lors de déconnexion réseau | IndexedDB + File d'attente |
| **Édition collaborative (Yjs)** | Synchronisation temps réel | Yjs (CRDT) + WebSocket |
| **Gestion de présence** | Partage de position du curseur | API Awareness + `throttleTime` |

### Prochaines étapes

- **[Traitement de données temps réel](./real-time-data.md)** - Implémentation détaillée WebSocket
- **[Gestion d'erreurs pratique](./error-handling-patterns.md)** - Traitement d'erreurs de communication API
- **[Stratégies de cache](./caching-strategies.md)** - Gestion du cache de données

## Ressources de référence

### Spécifications standards

- [RFC 6902 : JSON Patch](https://datatracker.ietf.org/doc/html/rfc6902) - Spécification JSON Patch
- [RFC 6901 : JSON Pointer](https://datatracker.ietf.org/doc/html/rfc6901) - Spécification JSON Pointer

### Bibliothèques

- [fast-json-patch](https://github.com/Starcounter-Jack/JSON-Patch) - Implémentation JSON Patch (conforme RFC)
- [Yjs](https://docs.yjs.dev/) - Implémentation CRDT (édition collaborative)
- [ShareDB](https://share.github.io/sharedb/) - Implémentation Operational Transform
- [Automerge](https://automerge.org/) - CRDT spécialisé JSON

### Ressources d'apprentissage

- [CRDTs: The Hard Parts](https://www.youtube.com/watch?v=x7drE24geUw) - Compréhension approfondie des CRDT (vidéo)
- [Operational Transformation Explained](https://operational-transformation.github.io/) - Explication détaillée OT
- [Real-time Collaborative Editing](https://pierrehedkvist.com/posts/1-creating-a-collaborative-editor) - Guide d'implémentation d'édition collaborative
