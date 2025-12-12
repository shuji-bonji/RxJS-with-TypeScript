---
description: Esta seção explica como combinar múltiplos Observables em sequência com a Creation Function concat e como utilizá-la para execução passo a passo e exibição de UI.
---

# concat - concatenar fluxos em sequência

`concat` é uma Creation Function que **executa sequencialmente** múltiplos Observables na ordem especificada.
O próximo Observable é emitido após o Observable anterior estar `complete`.

## Sintaxe básica e uso

```ts
import { concat, of, delay } from 'rxjs';

const obs1$ = of('A', 'B').pipe(delay(100));
const obs2$ = of('C', 'D').pipe(delay(100));

concat(obs1$, obs2$).subscribe(console.log);
// Saída: A → B → C → D
```

- Após todos os valores de `obs1$` terem sido emitidos, `obs2$` começará a ser emitido.
- O ponto chave é que os fluxos não são executados simultaneamente, mas "em ordem".

[🌐 Documentação Oficial RxJS - `concat`](https://rxjs.dev/api/index/function/concat)


## Padrões típicos de utilização

- **Processamento passo a passo**: Quando você deseja prosseguir para a próxima etapa após a etapa anterior estar completa.
- **Requisições de API com ordem garantida**: Operações assíncronas que precisam ser realizadas em sequência.
- **Controle de eventos de UI** onde a ordem é importante, como animações e notificações

## Exemplos práticos de código (com UI)

Este é um exemplo de **exibição de mensagens de carregamento e listas de dados em ordem sequencial**.

```ts
import { concat, of, timer } from 'rxjs';
import { map, take } from 'rxjs';

// Criar área de saída
const output = document.createElement('div');
output.innerHTML = '<h3>Exemplo prático de concat:</h3>';
document.body.appendChild(output);

// Fluxo de carregamento
const loading$ = timer(0, 1000).pipe(
  map((count) => `⏳ Carregando... (${count + 1}s)`),
  take(3) // Emite apenas por 3 segundos
);

// Fluxo de lista de dados
const data$ = of('🍎 Maçã', '🍌 Banana', '🍇 Uva');

// concat e exibir em ordem
concat(loading$, data$).subscribe((value) => {
  const item = document.createElement('div');
  item.textContent = value;
  output.appendChild(item);
});
```

- A mensagem de carregamento é exibida três vezes primeiro,
- A mensagem de carregamento é exibida três vezes primeiro, e então a lista de dados é exibida em ordem.
- Ao usar **concat**, uma exibição natural "passo a passo" pode ser facilmente alcançada.


## Operadores Relacionados

- **[concatWith](/pt/guide/operators/combination/concatWith)** - Versão Pipeable Operator (usado em pipeline)
- **[concatMap](/pt/guide/operators/transformation/concatMap)** - mapeia e concatena cada valor sequencialmente
- **[merge](/pt/guide/creation-functions/combination/merge)** - Creation Function de concatenação paralela
