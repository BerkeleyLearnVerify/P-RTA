using Antlr4.Runtime.Tree;
using Plang.Compiler.TypeChecker.AST;
using Plang.Compiler.TypeChecker.AST.Declarations;
using Plang.Compiler.TypeChecker.AST.States;
using Plang.Compiler.TypeChecker.Types;
using Plang.Compiler.Util;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;

namespace Plang.Compiler.TypeChecker
{
    public class DeclarationVisitor : PParserBaseVisitor<object>
    {
        private readonly StackProperty<Machine> currentMachine = new StackProperty<Machine>();
        private readonly StackProperty<Scope> currentScope;
        private readonly ParseTreeProperty<IPDecl> nodesToDeclarations;

        private DeclarationVisitor(
            ITranslationErrorHandler handler,
            Scope topLevelScope,
            ParseTreeProperty<IPDecl> nodesToDeclarations)
        {
            Handler = handler;
            currentScope = new StackProperty<Scope>(topLevelScope);
            this.nodesToDeclarations = nodesToDeclarations;
        }

        private Scope CurrentScope => currentScope.Value;
        private Machine CurrentMachine => currentMachine.Value;
        private ITranslationErrorHandler Handler { get; }

        public static void PopulateDeclarations(
            ITranslationErrorHandler handler,
            Scope topLevelScope,
            PParser.ProgramContext context,
            ParseTreeProperty<IPDecl> nodesToDeclarations)
        {
            DeclarationVisitor visitor = new DeclarationVisitor(handler, topLevelScope, nodesToDeclarations);
            visitor.Visit(context);
        }

        #region Events

        public override object VisitEventDecl(PParser.EventDeclContext context)
        {
            // EVENT name=Iden
            PEvent pEvent = (PEvent)nodesToDeclarations.Get(context);

            // cardinality?
            bool hasAssume = context.cardinality()?.ASSUME() != null;
            bool hasAssert = context.cardinality()?.ASSERT() != null;
            int cardinality = int.Parse(context.cardinality()?.IntLiteral().GetText() ?? "-1");
            pEvent.Assume = hasAssume ? cardinality : -1;
            pEvent.Assert = hasAssert ? cardinality : -1;

            // priority?
            bool hasPriority = context.priority()?.PRIORITY() != null;
            int priority = int.Parse(context.priority()?.IntLiteral().GetText() ?? "-1");
            pEvent.Priority = hasPriority ? priority : 0;

            // (COLON type)?
            pEvent.PayloadType = ResolveType(context.type());

            // SEMI
            return pEvent;
        }

        #endregion Events

        #region Interfaces

        public override object VisitInterfaceDecl(PParser.InterfaceDeclContext context)
        {
            // TYPE name=Iden
            Interface mInterface = (Interface)nodesToDeclarations.Get(context);

            // LPAREN type? RPAREN
            mInterface.PayloadType = ResolveType(context.type());

            IEventSet eventSet;
            if (context.RECEIVES() == null)
            {
                eventSet = CurrentScope.UniversalEventSet;
            }
            else
            {
                eventSet = new EventSet();
                if (context.nonDefaultEventList()?._events is IList<PParser.NonDefaultEventContext> events)
                {
                    foreach (PParser.NonDefaultEventContext eventContext in events)
                    {
                        eventSet.AddEvent((PEvent)Visit(eventContext));
                    }
                }
            }

            mInterface.ReceivableEvents = eventSet;
            return mInterface;
        }

        #endregion Interfaces

        private PLanguageType ResolveType(PParser.TypeContext typeContext)
        {
            return TypeResolver.ResolveType(typeContext, CurrentScope, Handler);
        }

        private Function CreateAnonFunction(PParser.AnonEventHandlerContext context)
        {
            Function fun = new Function(context)
            {
                Owner = CurrentMachine,
                Scope = CurrentScope.MakeChildScope()
            };

            CurrentMachine.AddMethod(fun);

            if (context.funParam() is PParser.FunParamContext paramContext)
            {
                Variable param = fun.Scope.Put(paramContext.name.GetText(), paramContext, VariableRole.Param);
                param.Type = ResolveType(paramContext.type());
                nodesToDeclarations.Put(paramContext, param);
                fun.Signature.Parameters.Add(param);
            }

            nodesToDeclarations.Put(context, fun);
            return fun;
        }

        private Function CreateAnonFunction(PParser.NoParamAnonEventHandlerContext context)
        {
            Function fun = new Function(context)
            {
                Owner = CurrentMachine,
                Scope = CurrentScope.MakeChildScope()
            };

            CurrentMachine.AddMethod(fun);

            nodesToDeclarations.Put(context, fun);
            return fun;
        }

        private State FindState(PParser.StateNameContext context)
        {
            Scope scope = CurrentMachine.Scope;
            foreach (PParser.IdenContext groupToken in context._groups)
            {
                if (!scope.Get(groupToken.GetText(), out StateGroup group))
                {
                    throw Handler.MissingDeclaration(groupToken, "group", groupToken.GetText());
                }

                scope = group.Scope;
            }

            if (!scope.Get(context.state.GetText(), out State state))
            {
                throw Handler.MissingDeclaration(context.state, "state", context.state.GetText());
            }

            return state;
        }

        #region Typedefs

        public override object VisitForeignTypeDef(PParser.ForeignTypeDefContext context)
        {
            // TYPE name=iden
            TypeDef typedef = (TypeDef)nodesToDeclarations.Get(context);
            // SEMI
            typedef.Type = new ForeignType(typedef.Name);
            return typedef;
        }

        public override object VisitPTypeDef(PParser.PTypeDefContext context)
        {
            // TYPE name=iden
            TypeDef typedef = (TypeDef)nodesToDeclarations.Get(context);
            // ASSIGN type
            typedef.Type = ResolveType(context.type());
            // SEMI
            return typedef;
        }

        #endregion Typedefs

        #region Enum typedef

        public override object VisitEnumTypeDefDecl(PParser.EnumTypeDefDeclContext context)
        {
            // ENUM name=iden
            PEnum pEnum = (PEnum)nodesToDeclarations.Get(context);

            // LBRACE enumElemList RBRACE
            if (context.enumElemList() is PParser.EnumElemListContext elemList)
            {
                EnumElem[] elems = (EnumElem[])Visit(elemList);
                for (int i = 0; i < elems.Length; i++)
                {
                    EnumElem elem = elems[i];
                    elem.Value = i;
                    pEnum.AddElement(elem);
                }
            }
            // | LBRACE numberedEnumElemList RBRACE
            else if (context.numberedEnumElemList() is PParser.NumberedEnumElemListContext numberedElemList)
            {
                EnumElem[] numberedElems = (EnumElem[])Visit(numberedElemList);
                foreach (EnumElem elem in numberedElems)
                {
                    pEnum.AddElement(elem);
                }
            }
            else
            {
                Debug.Fail("grammar requires enum declarations to have element lists");
            }

            return pEnum;
        }

        public override object VisitEnumElemList(PParser.EnumElemListContext context)
        {
            // enumElem (COMMA enumElem)*
            return context.enumElem().Select(Visit).Cast<EnumElem>().ToArray();
        }

        public override object VisitEnumElem(PParser.EnumElemContext context)
        {
            // name=iden
            return (EnumElem)nodesToDeclarations.Get(context);
        }

        public override object VisitNumberedEnumElemList(PParser.NumberedEnumElemListContext context)
        {
            // numberedEnumElem (COMMA numberedEnumElem)*
            return context.numberedEnumElem().Select(Visit).Cast<EnumElem>().ToArray();
        }

        public override object VisitNumberedEnumElem(PParser.NumberedEnumElemContext context)
        {
            // name=iden ASSIGN value=IntLiteral
            EnumElem elem = (EnumElem)nodesToDeclarations.Get(context);
            elem.Value = int.Parse(context.value.Text);
            return elem;
        }

        #endregion Enum typedef

        #region Event sets

        public override object VisitEventSetDecl(PParser.EventSetDeclContext context)
        {
            // EVENTSET name=iden
            NamedEventSet es = (NamedEventSet)nodesToDeclarations.Get(context);
            // ASSIGN LBRACE eventSetLiteral RBRACE
            es.AddEvents((PEvent[])Visit(context.eventSetLiteral()));
            // SEMI
            return es;
        }

        public override object VisitEventSetLiteral(PParser.EventSetLiteralContext context)
        {
            // events+=nonDefaultEvent (COMMA events+=nonDefaultEvent)*
            return context._events.Select(Visit).Cast<PEvent>().ToArray();
        }

        public override object VisitNonDefaultEvent(PParser.NonDefaultEventContext context)
        {
            // HALT | iden
            string eventName = context.GetText();
            if (!CurrentScope.Lookup(eventName, out PEvent pEvent))
            {
                throw Handler.MissingDeclaration(context, "event", eventName);
            }

            return pEvent;
        }

        #endregion Event sets

        #region Machines

        public override object VisitImplMachineDecl(PParser.ImplMachineDeclContext context)
        {
            // MACHINE name=iden
            Machine machine = (Machine)nodesToDeclarations.Get(context);

            // cardinality?
            bool hasAssume = context.cardinality()?.ASSUME() != null;
            bool hasAssert = context.cardinality()?.ASSERT() != null;
            long cardinality = long.Parse(context.cardinality()?.IntLiteral().GetText() ?? "-1");
            if (cardinality > uint.MaxValue)
            {
                throw Handler.ValueOutOfRange(context.cardinality(), "uint32");
            }

            machine.Assume = hasAssume ? (uint?)cardinality : null;
            machine.Assert = hasAssert ? (uint?)cardinality : null;

            // priority?
            bool hasPriority = context.priority()?.PRIORITY() != null;
            long priority = int.Parse(context.priority()?.IntLiteral().GetText() ?? "-1");
            if (priority > uint.MaxValue)
            {
                throw Handler.ValueOutOfRange(context.priority(), "uint32");
            }

            machine.Priority = hasPriority ? (uint?)priority : null;

            // receivesSends*
            foreach (PParser.ReceivesSendsContext receivesSends in context.receivesSends())
            {
                Tuple<string, PEvent[]> recvSendTuple = (Tuple<string, PEvent[]>)Visit(receivesSends);
                string eventSetType = recvSendTuple.Item1;
                if (eventSetType.Equals("RECV", StringComparison.InvariantCulture))
                {
                    if (machine.Receives == null)
                    {
                        machine.Receives = new EventSet();
                    }

                    foreach (PEvent @event in recvSendTuple.Item2)
                    {
                        machine.Receives.AddEvent(@event);
                    }
                }
                else if (eventSetType.Equals("SEND", StringComparison.InvariantCulture))
                {
                    if (machine.Sends == null)
                    {
                        machine.Sends = new EventSet();
                    }

                    foreach (PEvent @event in recvSendTuple.Item2)
                    {
                        machine.Sends.AddEvent(@event);
                    }
                }
                else
                {
                    Debug.Fail("grammar changed surrounding receives/sends.");
                }
            }

            if (machine.Receives == null)
            {
                machine.Receives = CurrentScope.UniversalEventSet;
            }

            if (machine.Sends == null)
            {
                machine.Sends = CurrentScope.UniversalEventSet;
            }

            // machineBody
            using (currentScope.NewContext(machine.Scope))
            using (currentMachine.NewContext(machine))
            {
                Visit(context.machineBody());
            }

            // initialize the corresponding interface
            currentScope.Value.Get(machine.Name, out Interface @interface);
            @interface.ReceivableEvents = machine.Receives;
            @interface.PayloadType = machine.PayloadType;

            return machine;
        }

        public override object VisitMachineReceive(PParser.MachineReceiveContext context)
        {
            PEvent[] events = context.eventSetLiteral() == null
                ? new PEvent[0]
                : (PEvent[])Visit(context.eventSetLiteral());
            return Tuple.Create("RECV", events);
        }

        public override object VisitMachineSend(PParser.MachineSendContext context)
        {
            PEvent[] events = context.eventSetLiteral() == null
                ? new PEvent[0]
                : (PEvent[])Visit(context.eventSetLiteral());
            return Tuple.Create("SEND", events);
        }

        public override object VisitSpecMachineDecl(PParser.SpecMachineDeclContext context)
        {
            // SPEC name=Iden
            Machine specMachine = (Machine)nodesToDeclarations.Get(context);

            // spec machines neither send nor receive events.
            specMachine.Receives = new EventSet();
            specMachine.Sends = new EventSet();

            // OBSERVES eventSetLiteral
            specMachine.Observes = new EventSet();
            foreach (PEvent pEvent in (PEvent[])Visit(context.eventSetLiteral()))
            {
                specMachine.Observes.AddEvent(pEvent);
            }

            // machineBody
            using (currentScope.NewContext(specMachine.Scope))
            using (currentMachine.NewContext(specMachine))
            {
                Visit(context.machineBody());
            }

            return specMachine;
        }

        public override object VisitMachineBody(PParser.MachineBodyContext context)
        {
            foreach (PParser.MachineEntryContext machineEntryContext in context.machineEntry())
            {
                switch (Visit(machineEntryContext))
                {
                    case Variable[] fields:
                        CurrentMachine.AddFields(fields);
                        break;

                    case Function method:
                        CurrentMachine.AddMethod(method);
                        break;

                    case State state:
                        CurrentMachine.AddState(state);
                        break;

                    case StateGroup group:
                        CurrentMachine.AddGroup(group);
                        break;

                    default:
                        throw Handler.InternalError(machineEntryContext,
                            new ArgumentOutOfRangeException(nameof(context)));
                }
            }

            return null;
        }

        public override object VisitMachineEntry(PParser.MachineEntryContext context)
        {
            IParseTree subExpr = context.varDecl() ??
                          context.funDecl() ??
                          (IParseTree)context.group() ??
                          context.stateDecl() ??
                          throw Handler.InternalError(context, new ArgumentOutOfRangeException(nameof(context)));
            return Visit(subExpr);
        }

        public override object VisitVarDecl(PParser.VarDeclContext context)
        {
            // COLON type
            PLanguageType variableType = ResolveType(context.type());

            // VAR idenList
            Variable[] variables = new Variable[context.idenList()._names.Count];
            IList<PParser.IdenContext> varNameCtxs = context.idenList()._names;
            for (int i = 0; i < varNameCtxs.Count; i++)
            {
                Variable variable = (Variable)nodesToDeclarations.Get(varNameCtxs[i]);
                variable.Type = variableType;
                variables[i] = variable;
            }

            // SEMI
            return variables;
        }

        public override object VisitGroup(PParser.GroupContext context)
        {
            StateGroup group = (StateGroup)nodesToDeclarations.Get(context);
            group.OwningMachine = CurrentMachine;
            using (currentScope.NewContext(group.Scope))
            {
                foreach (PParser.GroupItemContext groupItemContext in context.groupItem())
                {
                    switch (Visit(groupItemContext))
                    {
                        case StateGroup subGroup:
                            group.AddGroup(subGroup);
                            break;

                        case State state:
                            group.AddState(state);
                            break;

                        default:
                            throw Handler.InternalError(groupItemContext,
                                new ArgumentOutOfRangeException(nameof(context)));
                    }
                }
            }

            return group;
        }

        public override object VisitGroupItem(PParser.GroupItemContext context)
        {
            IParseTree item = (IParseTree)context.stateDecl() ??
                       context.group() ??
                       throw Handler.InternalError(context, new ArgumentOutOfRangeException(nameof(context)));
            return Visit(item);
        }

        public override object VisitStateDecl(PParser.StateDeclContext context)
        {
            // STATE name=iden
            State state = (State)nodesToDeclarations.Get(context);
            state.OwningMachine = CurrentMachine;

            // START?
            state.IsStart = context.START() != null;

            // temperature=(HOT | COLD)?
            state.Temperature = context.temperature == null
                ? StateTemperature.Warm
                : context.HOT() != null
                    ? StateTemperature.Hot
                    : StateTemperature.Cold;

            // LBRACE stateBodyItem* RBRACE ;
            foreach (PParser.StateBodyItemContext stateBodyItemContext in context.stateBodyItem())
            {
                switch (Visit(stateBodyItemContext))
                {
                    case IStateAction[] actions:
                        foreach (IStateAction action in actions)
                        {
                            if (state.HasHandler(action.Trigger))
                            {
                                throw Handler.DuplicateEventAction(action.SourceLocation, state[action.Trigger], state);
                            }

                            if (action.Trigger.Name.Equals("null") && CurrentMachine.IsSpec)
                            {
                                throw Handler.NullTransitionInMonitor(action.SourceLocation, CurrentMachine);
                            }

                            state[action.Trigger] = action;
                        }

                        break;

                    case Tuple<string, Function> entryOrExit:
                        if (entryOrExit.Item1.Equals("ENTRY"))
                        {
                            if (state.Entry != null)
                            {
                                throw Handler.DuplicateStateEntry(stateBodyItemContext, state.Entry, state);
                            }

                            state.Entry = entryOrExit.Item2;
                        }
                        else
                        {
                            if (state.Exit != null)
                            {
                                throw Handler.DuplicateStateExitHandler(stateBodyItemContext, state.Exit, state);
                            }

                            state.Exit = entryOrExit.Item2;
                        }

                        break;

                    case Tuple<PParser.EventDrivenRTAModuleContext, IStateAction[]> eventDrivenRTAModule:
                        if (state.RTAModule != null)
                        {
                            throw Handler.DuplicateRTAModule(stateBodyItemContext, state, state.RTAModule);
                        }
                        state.RTAModule = eventDrivenRTAModule.Item1;
                        foreach (IStateAction action in eventDrivenRTAModule.Item2)
                        {
                            if (state.HasHandler(action.Trigger))
                            {
                                throw Handler.DuplicateEventAction(action.SourceLocation, state[action.Trigger], state);
                            }

                            if (action.Trigger.Name.Equals("null") && CurrentMachine.IsSpec)
                            {
                                throw Handler.NullTransitionInMonitor(action.SourceLocation, CurrentMachine);
                            }

                            state[action.Trigger] = action;
                        }
                        break;

                    case Tuple<PParser.TimeDrivenRTAModuleContext, Function> timeDrivenRTAModule:
                        if (state.RTAModule != null)
                        {
                            throw Handler.DuplicateRTAModule(stateBodyItemContext, state, state.RTAModule);
                        }
                        state.RTAModule = timeDrivenRTAModule.Item1;
                        state.TimeDrivenRTAModule = timeDrivenRTAModule.Item2;
                        break;

                    default:
                        throw Handler.InternalError(stateBodyItemContext,
                            new ArgumentOutOfRangeException(nameof(context)));
                }
            }

            if (state.IsStart)
            {
                if (CurrentMachine.StartState != null)
                {
                    throw Handler.DuplicateStartState(context, state, CurrentMachine.StartState, CurrentMachine);
                }

                CurrentMachine.StartState = state;
                CurrentMachine.PayloadType =
                    state.Entry?.Signature.Parameters.ElementAtOrDefault(0)?.Type ?? PrimitiveType.Null;
            }

            if (state.Entry == null && state.RTAModule != null) {
                Function fun = new Function(state.RTAModule)
                {
                    Owner = CurrentMachine,
                    Scope = CurrentScope.MakeChildScope()
                };
                CurrentMachine.AddMethod(fun);
                nodesToDeclarations.Put(context, fun);
                fun.Role |= FunctionRole.EntryHandler;
                state.Entry = fun;
            }

            return state;
        }

        public override object VisitStateEntry(PParser.StateEntryContext context)
        {
            Function fun;
            if (context.anonEventHandler() != null)
            {
                fun = CreateAnonFunction(context.anonEventHandler());
            }
            else
            {
                string funName = context.funName.GetText();
                if (!CurrentScope.Lookup(funName, out fun))
                {
                    throw Handler.MissingDeclaration(context.funName, "function", funName);
                }
            }

            fun.Role |= FunctionRole.EntryHandler;

            return Tuple.Create("ENTRY", fun);
        }

        public override object VisitStateExit(PParser.StateExitContext context)
        {
            Function fun;
            if (context.noParamAnonEventHandler() != null)
            {
                fun = CreateAnonFunction(context.noParamAnonEventHandler());
            }
            else
            {
                string funName = context.funName.GetText();
                if (!CurrentScope.Lookup(funName, out fun))
                {
                    throw Handler.MissingDeclaration(context.funName, "function", funName);
                }
            }

            fun.Role |= FunctionRole.ExitHandler;
            return Tuple.Create("EXIT", fun);
        }

        public override object VisitEventDrivenRTAModule(PParser.EventDrivenRTAModuleContext context)
        {

            List<PParser.EventDrivenControllerContext> controllerContexts = context.eventDrivenRTAModuleBody().eventDrivenController().ToList();
            PParser.DecisionModuleContext decisionmoduleContext = context.eventDrivenRTAModuleBody().decisionModule();
            string decisionmoduleFunName = decisionmoduleContext.funName.GetText();
            List<PParser.DecisionModulePeriodContext> decisionPeriodContexts = decisionmoduleContext.decisionModulePeriods().decisionModulePeriod().ToList();
            List<Function> functions = CurrentScope.GetAllMethods().ToList();

            List<Function> rtaControllers = new List<Function>();

            List<string> rtaControllerNames = new List<string>();

            Dictionary<string, int> rtaDecisionPeriods = new Dictionary<string, int>();

            for (int i = 0; i < controllerContexts.Count; i++)
            {
                PParser.EventDrivenControllerContext controllerContext = controllerContexts[i];
                // Check if controller is a function
                string controllerFunName = controllerContext.funName.GetText();
                if (!CurrentScope.Lookup(controllerFunName, out Function _))
                {
                    throw Handler.MissingDeclaration(controllerContext.funName, "function", controllerFunName);
                }
                // Check if parameter and return type of controller is null
                Function controllerFunction = functions.Find(x => x.Name.Equals(controllerFunName));
                if (!controllerFunction.Signature.ParameterTypes.ToList().Count.Equals(0))
                {
                    throw Handler.RTATypeError(controllerContext, "controller", "parameter", "null");
                }
                if (!controllerFunction.Signature.ReturnType.IsSameTypeAs(PrimitiveType.Null))
                {
                    throw Handler.RTATypeError(controllerContext, "controller", "return", "null");
                }
                rtaControllers.Add(controllerFunction);
                rtaControllerNames.Add(controllerFunName);
                // Check if controller duplicates any previously declared controller
                for (int j = 0; j < i; j++) {
                    PParser.EventDrivenControllerContext duplicatedControllerContext = controllerContexts[j];
                    if (duplicatedControllerContext.funName.GetText().Equals(controllerFunName))
                    {
                        throw Handler.DuplicateController(controllerContext, "controller", duplicatedControllerContext);
                    }
                }
                // Check if decision period of controller is defined
                if (!decisionPeriodContexts.Exists(x => x.funName.GetText().Equals(controllerFunName)))
                {
                    throw Handler.ControllerDecisionPeriodNotFound(decisionmoduleContext, controllerContext);
                }
                // Check if decisionmodule duplicates any controller
                if (controllerFunName.Equals(decisionmoduleFunName))
                {
                    throw Handler.DuplicateController(decisionmoduleContext, "decisionmodule", controllerContext);
                }
            }

            // Check if decisionmodule is a function
            if (!CurrentScope.Lookup(decisionmoduleFunName, out Function _))
            {
                throw Handler.MissingDeclaration(decisionmoduleContext.funName, "function", decisionmoduleFunName);
            }

            for (int i = 0; i < decisionPeriodContexts.Count; i++)
            {
                PParser.DecisionModulePeriodContext decisionPeriodContext = decisionPeriodContexts[i];
                string controllerFunName = decisionPeriodContext.funName.GetText();
                // Check if given controllers are declared
                if (!controllerContexts.Exists(x => x.funName.GetText().Equals(controllerFunName)))
                {
                    throw Handler.MissingDeclaration(decisionPeriodContext.funName, "controller", controllerFunName);
                }
                // Check if decision periods of controllers are unique
                for (int j = 0; j < i; j++)
                {
                    if (decisionPeriodContexts[j].funName.GetText().Equals(controllerFunName))
                    {
                        throw Handler.DuplicateDecisionPeriod(decisionmoduleContext, controllerFunName);
                    }
                }
                rtaDecisionPeriods.Add(controllerFunName, int.Parse(decisionPeriodContext.decisionPeriod.Text));

            }

            Function decisionmoduleFunction = functions.Find(x => x.Name.Equals(decisionmoduleFunName));
            // Check if parameter type of decisionmodule is null
            if (!decisionmoduleFunction.Signature.ParameterTypes.ToList().Count.Equals(0))
            {
                throw Handler.RTATypeError(decisionmoduleContext, "decisionmodule", "parameter", "null");
            }
            // Check if return type of decisionmodule is string
            if (!decisionmoduleFunction.Signature.ReturnType.IsSameTypeAs(PrimitiveType.String))
            {
                throw Handler.RTATypeError(decisionmoduleContext, "decisionmodule", "return", "string");
            }

            // Search all paths of the decisionmodule function and check return strings
            PParser.PFunDeclContext decisionmodulefunDeclContext = (PParser.PFunDeclContext)decisionmoduleFunction.SourceLocation;
            List<string> returnedControllerFunNames = new List<string>();
            Stack<Antlr4.Runtime.ParserRuleContext> ruleStack = new Stack<Antlr4.Runtime.ParserRuleContext>();
            ruleStack.Push(decisionmodulefunDeclContext.functionBody());
            while (ruleStack.Count > 0)
            {
                Antlr4.Runtime.ParserRuleContext top = ruleStack.Pop();
                if (top is PParser.ReturnStmtContext)
                {
                    // Check if return string is an explicit string, e.g., return "AC"
                    if (!(top.GetChild(1) is PParser.StringExprContext))
                    {
                        throw Handler.ImplicitControllerNameReturn(top);
                    }
                    PParser.StringExprContext formatedStringContext = (PParser.StringExprContext)top.GetChild(1);
                    string controllerFunName = formatedStringContext.formatedString().StringLiteral().GetText();
                    controllerFunName = controllerFunName.Substring(1, controllerFunName.Length - 2);
                    // Check if return string is not a formatted string
                    if (TypeCheckingUtils.PrintStmtNumArgs(controllerFunName) > 0)
                    {
                        throw Handler.ImplicitControllerNameReturn(top);
                    }
                    // Check if the return string is a controller
                    if (!controllerContexts.Exists(x => x.funName.GetText().Equals(controllerFunName)))
                    {
                        throw Handler.ReturnValueIsNotAController(top);
                    }
                    returnedControllerFunNames.Add(controllerFunName);

                }
                else
                {
                    for (int i = 0; i < top.ChildCount; i++)
                    {
                        if (!(top.GetChild(i) is ITerminalNode))
                        {
                            ruleStack.Push((Antlr4.Runtime.ParserRuleContext)top.GetChild(i));
                        }
                    }
                }
            }
            // Check if all controllers are in at least one of the return statements of decisionmodule function
            for (int i = 0; i < controllerContexts.Count; i++)
            {
                string controllerFunName = controllerContexts[i].funName.GetText();
                if (!returnedControllerFunNames.Contains(controllerContexts[i].funName.GetText()))
                {
                    throw Handler.MissingControllerStringReturn(decisionmodulefunDeclContext, controllerFunName);
                }
            }

            List<IStateAction> actions = new List<IStateAction>();
            List<PParser.TriggerContext> triggers = context.eventDrivenRTAModuleBody().trigger().ToList();
            foreach (PParser.TriggerContext trigger in triggers)
            {
                Function eventHandler;
                if (trigger.funName != null)
                {
                    string funName = trigger.funName.GetText();
                    if (!CurrentScope.Lookup(funName, out eventHandler))
                    {
                        throw Handler.MissingDeclaration(trigger.funName, "function", funName);
                    }
                    eventHandler.Role |= FunctionRole.EventHandler;
                }
                else
                {
                    eventHandler = null;
                }

                Function fun = new Function(context)
                {
                    Owner = CurrentMachine,
                    Scope = CurrentScope.MakeChildScope()
                };
                CurrentMachine.AddMethod(fun);

                if (eventHandler != null && eventHandler.Signature.Parameters.Count > 0)
                {
                    Variable eventHandlerParam = eventHandler.Signature.Parameters[0];
                    Variable param = fun.Scope.Put(eventHandlerParam.Name, context, VariableRole.Param);
                    param.Type = eventHandlerParam.Type;
                    nodesToDeclarations.Put(context, param);
                    fun.Signature.Parameters.Add(param);
                }

                nodesToDeclarations.Put(context, fun);

                fun.Role |= FunctionRole.RTAModule;
                fun.RTADecisionModule = decisionmoduleFunction;
                fun.RTAControllers = rtaControllers;
                fun.RTAControllerNames = rtaControllerNames;
                fun.RTADecisionPeriods = rtaDecisionPeriods;
                fun.RTAEventHandler = eventHandler;

                // ON eventList
                foreach (PParser.EventIdContext eventIdContext in trigger.eventList().eventId())
                {
                    if (!CurrentScope.Lookup(eventIdContext.GetText(), out PEvent evt))
                    {
                        throw Handler.MissingDeclaration(eventIdContext, "event", eventIdContext.GetText());
                    }
                    actions.Add(new EventDoAction(eventIdContext, evt, fun));
                }
            }

            if (CurrentMachine.isStartedVar == null)
            {
                Variable isStarted = new Variable("", context, VariableRole.Field);
                isStarted.Type = PrimitiveType.Bool;
                CurrentMachine.AddField(isStarted);
                CurrentMachine.isStartedVar = isStarted;
            }

            if (CurrentMachine.modeVar == null)
            {
                Variable mode = new Variable("", context, VariableRole.Field);
                mode.Type = PrimitiveType.String;
                CurrentMachine.AddField(mode);
                CurrentMachine.modeVar = mode;
            }

            if (CurrentMachine.decisionPeriodVar == null)
            {
                Variable decisionPeriod = new Variable("", context, VariableRole.Field);
                decisionPeriod.Type = PrimitiveType.Int;
                CurrentMachine.AddField(decisionPeriod);
                CurrentMachine.decisionPeriodVar = decisionPeriod;
            }

            if (CurrentMachine.decisionPeriodCountVar == null)
            {
                Variable decisionPeriodCount = new Variable("", context, VariableRole.Field);
                decisionPeriodCount.Type = PrimitiveType.Int;
                CurrentMachine.AddField(decisionPeriodCount);
                CurrentMachine.decisionPeriodCountVar = decisionPeriodCount;
            }

            return Tuple.Create(context, actions.ToArray());
        }

        public override object VisitTimeDrivenRTAModule(PParser.TimeDrivenRTAModuleContext context)
        {

            List<PParser.TimeDrivenControllerContext> controllerContexts = context.timeDrivenRTAModuleBody().timeDrivenController().ToList();
            PParser.DecisionModuleContext decisionmoduleContext = context.timeDrivenRTAModuleBody().decisionModule();
            string decisionmoduleFunName = decisionmoduleContext.funName.GetText();
            List<PParser.DecisionModulePeriodContext> decisionPeriodContexts = decisionmoduleContext.decisionModulePeriods().decisionModulePeriod().ToList();
            List<Function> functions = CurrentScope.GetAllMethods().ToList();

            List<Function> rtaControllers = new List<Function>();

            List<string> rtaControllerNames = new List<string>();

            Dictionary<string, int> rtaDecisionPeriods = new Dictionary<string, int>();

            Dictionary<string, Tuple<int, int>> rtaControllerPeriods = new Dictionary<string, Tuple<int, int>>();

            for (int i = 0; i < controllerContexts.Count; i++)
            {
                PParser.TimeDrivenControllerContext controllerContext = controllerContexts[i];
                // Check if controller is a function
                string controllerFunName = controllerContext.funName.GetText();
                if (!CurrentScope.Lookup(controllerFunName, out Function _))
                {
                    throw Handler.MissingDeclaration(controllerContext.funName, "function", controllerFunName);
                }
                // Check if parameter and return type of controller is null
                Function controllerFunction = functions.Find(x => x.Name.Equals(controllerFunName));
                if (!controllerFunction.Signature.ParameterTypes.ToList().Count.Equals(0))
                {
                    throw Handler.RTATypeError(controllerContext, "controller", "parameter", "null");
                }
                if (!controllerFunction.Signature.ReturnType.IsSameTypeAs(PrimitiveType.Null))
                {
                    throw Handler.RTATypeError(controllerContext, "controller", "return", "null");
                }
                rtaControllers.Add(controllerFunction);
                rtaControllerNames.Add(controllerFunName);
                //Console.WriteLine(controllerContext.period().timeUnit().GetText());
                string timeUnit = controllerContext.period().timeUnit().GetText();
                if (timeUnit.Equals("s"))
                {
                    rtaControllerPeriods.Add(controllerFunName, Tuple.Create(int.Parse(controllerContext.period().IntLiteral().GetText()), 9));
                }
                else if (timeUnit.Equals("ms"))
                {
                    rtaControllerPeriods.Add(controllerFunName, Tuple.Create(int.Parse(controllerContext.period().IntLiteral().GetText()), 6));
                }
                else if (timeUnit.Equals("us"))
                {
                    rtaControllerPeriods.Add(controllerFunName, Tuple.Create(int.Parse(controllerContext.period().IntLiteral().GetText()), 3));
                }
                else if (timeUnit.Equals("ns"))
                {
                    rtaControllerPeriods.Add(controllerFunName, Tuple.Create(int.Parse(controllerContext.period().IntLiteral().GetText()), 0));
                }
                // Check if controller duplicates any previously declared controller
                for (int j = 0; j < i; j++)
                {
                    PParser.TimeDrivenControllerContext duplicatedControllerContext = controllerContexts[j];
                    if (duplicatedControllerContext.funName.GetText().Equals(controllerFunName))
                    {
                        throw Handler.DuplicateController(controllerContext, "controller", duplicatedControllerContext);
                    }
                }
                // Check if decision period of controller is defined
                if (!decisionPeriodContexts.Exists(x => x.funName.GetText().Equals(controllerFunName)))
                {
                    throw Handler.ControllerDecisionPeriodNotFound(decisionmoduleContext, controllerContext);
                }
                // Check if decisionmodule duplicates any controller
                if (controllerFunName.Equals(decisionmoduleFunName))
                {
                    throw Handler.DuplicateController(decisionmoduleContext, "decisionmodule", controllerContext);
                }
            }

            // Check if decisionmodule is a function
            if (!CurrentScope.Lookup(decisionmoduleFunName, out Function _))
            {
                throw Handler.MissingDeclaration(decisionmoduleContext.funName, "function", decisionmoduleFunName);
            }

            for (int i = 0; i < decisionPeriodContexts.Count; i++)
            {
                PParser.DecisionModulePeriodContext decisionPeriodContext = decisionPeriodContexts[i];
                string controllerFunName = decisionPeriodContext.funName.GetText();
                // Check if given controllers are declared
                if (!controllerContexts.Exists(x => x.funName.GetText().Equals(controllerFunName)))
                {
                    throw Handler.MissingDeclaration(decisionPeriodContext.funName, "controller", controllerFunName);
                }
                // Check if decision periods of controllers are unique
                for (int j = 0; j < i; j++)
                {
                    if (decisionPeriodContexts[j].funName.GetText().Equals(controllerFunName))
                    {
                        throw Handler.DuplicateDecisionPeriod(decisionmoduleContext, controllerFunName);
                    }
                }
                rtaDecisionPeriods.Add(controllerFunName, int.Parse(decisionPeriodContext.decisionPeriod.Text));

            }

            Function decisionmoduleFunction = functions.Find(x => x.Name.Equals(decisionmoduleFunName));
            // Check if parameter type of decisionmodule is null
            if (!decisionmoduleFunction.Signature.ParameterTypes.ToList().Count.Equals(0))
            {
                throw Handler.RTATypeError(decisionmoduleContext, "decisionmodule", "parameter", "null");
            }
            // Check if return type of decisionmodule is string
            if (!decisionmoduleFunction.Signature.ReturnType.IsSameTypeAs(PrimitiveType.String))
            {
                throw Handler.RTATypeError(decisionmoduleContext, "decisionmodule", "return", "string");
            }

            // Search all paths of the decisionmodule function and check return strings
            PParser.PFunDeclContext decisionmodulefunDeclContext = (PParser.PFunDeclContext)decisionmoduleFunction.SourceLocation;
            List<string> returnedControllerFunNames = new List<string>();
            Stack<Antlr4.Runtime.ParserRuleContext> ruleStack = new Stack<Antlr4.Runtime.ParserRuleContext>();
            ruleStack.Push(decisionmodulefunDeclContext.functionBody());
            while (ruleStack.Count > 0)
            {
                Antlr4.Runtime.ParserRuleContext top = ruleStack.Pop();
                if (top is PParser.ReturnStmtContext)
                {
                    // Check if return string is an explicit string, e.g., return "AC"
                    if (!(top.GetChild(1) is PParser.StringExprContext))
                    {
                        throw Handler.ImplicitControllerNameReturn(top);
                    }
                    PParser.StringExprContext formatedStringContext = (PParser.StringExprContext)top.GetChild(1);
                    string controllerFunName = formatedStringContext.formatedString().StringLiteral().GetText();
                    controllerFunName = controllerFunName.Substring(1, controllerFunName.Length - 2);
                    // Check if return string is not a formatted string
                    if (TypeCheckingUtils.PrintStmtNumArgs(controllerFunName) > 0)
                    {
                        throw Handler.ImplicitControllerNameReturn(top);
                    }
                    // Check if the return string is a controller
                    if (!controllerContexts.Exists(x => x.funName.GetText().Equals(controllerFunName)))
                    {
                        throw Handler.ReturnValueIsNotAController(top);
                    }
                    returnedControllerFunNames.Add(controllerFunName);

                }
                else
                {
                    for (int i = 0; i < top.ChildCount; i++)
                    {
                        if (!(top.GetChild(i) is ITerminalNode))
                        {
                            ruleStack.Push((Antlr4.Runtime.ParserRuleContext)top.GetChild(i));
                        }
                    }
                }
            }
            // Check if all controllers are in at least one of the return statements of decisionmodule function
            for (int i = 0; i < controllerContexts.Count; i++)
            {
                string controllerFunName = controllerContexts[i].funName.GetText();
                if (!returnedControllerFunNames.Contains(controllerContexts[i].funName.GetText()))
                {
                    throw Handler.MissingControllerStringReturn(decisionmodulefunDeclContext, controllerFunName);
                }
            }


            Function fun = new Function(context)
            {
                Owner = CurrentMachine,
                Scope = CurrentScope.MakeChildScope()
            };
            CurrentMachine.AddMethod(fun);
            nodesToDeclarations.Put(context, fun);

            fun.Role |= FunctionRole.RTAModule;
            fun.RTADecisionModule = decisionmoduleFunction;
            fun.RTAControllers = rtaControllers;
            fun.RTAControllerNames = rtaControllerNames;
            fun.RTAControllerPeriods = rtaControllerPeriods;
            fun.RTADecisionPeriods = rtaDecisionPeriods;

            if (CurrentMachine.isStartedVar == null)
            {
                Variable isStarted = new Variable("", context, VariableRole.Field);
                isStarted.Type = PrimitiveType.Bool;
                CurrentMachine.AddField(isStarted);
                CurrentMachine.isStartedVar = isStarted;
            }

            if (CurrentMachine.modeVar == null)
            {
                Variable mode = new Variable("", context, VariableRole.Field);
                mode.Type = PrimitiveType.String;
                CurrentMachine.AddField(mode);
                CurrentMachine.modeVar = mode;
            }

            if (CurrentMachine.decisionPeriodVar == null)
            {
                Variable decisionPeriod = new Variable("", context, VariableRole.Field);
                decisionPeriod.Type = PrimitiveType.Int;
                CurrentMachine.AddField(decisionPeriod);
                CurrentMachine.decisionPeriodVar = decisionPeriod;
            }

            if (CurrentMachine.decisionPeriodCountVar == null)
            {
                Variable decisionPeriodCount = new Variable("", context, VariableRole.Field);
                decisionPeriodCount.Type = PrimitiveType.Int;
                CurrentMachine.AddField(decisionPeriodCount);
                CurrentMachine.decisionPeriodCountVar = decisionPeriodCount;
            }

            if (CurrentMachine.periodVar == null)
            {
                Variable period = new Variable("", context, VariableRole.Field);
                period.Type = PrimitiveType.Int;
                CurrentMachine.AddField(period);
                CurrentMachine.periodVar = period;
            }

            if (CurrentMachine.periodUnitVar == null)
            {
                Variable periodUnit = new Variable("", context, VariableRole.Field);
                periodUnit.Type = PrimitiveType.Int;
                CurrentMachine.AddField(periodUnit);
                CurrentMachine.periodUnitVar = periodUnit;
            }

            List<IStateAction> actions = new List<IStateAction>();
            return Tuple.Create(context, fun);
        }

        public override object VisitStateDefer(PParser.StateDeferContext context)
        {
            if (CurrentMachine.IsSpec)
            {
                throw Handler.DeferredEventInMonitor(context, CurrentMachine);
            }

            // DEFER nonDefaultEventList
            IList<PParser.NonDefaultEventContext> eventContexts = context.nonDefaultEventList()._events;
            IStateAction[] actions = new IStateAction[eventContexts.Count];
            for (int i = 0; i < eventContexts.Count; i++)
            {
                PParser.NonDefaultEventContext token = eventContexts[i];
                if (!CurrentScope.Lookup(token.GetText(), out PEvent evt))
                {
                    throw Handler.MissingDeclaration(token, "event", token.GetText());
                }

                actions[i] = new EventDefer(token, evt);
            }

            return actions;
        }

        public override object VisitStateIgnore(PParser.StateIgnoreContext context)
        {
            // IGNORE nonDefaultEventList
            List<IStateAction> actions = new List<IStateAction>();
            foreach (PParser.NonDefaultEventContext token in context.nonDefaultEventList()._events)
            {
                if (!CurrentScope.Lookup(token.GetText(), out PEvent evt))
                {
                    throw Handler.MissingDeclaration(token, "event", token.GetText());
                }

                actions.Add(new EventIgnore(token, evt));
            }

            return actions.ToArray();
        }

        public override object VisitOnEventDoAction(PParser.OnEventDoActionContext context)
        {
            Function fun;
            if (context.anonEventHandler() is PParser.AnonEventHandlerContext anonEventHandler)
            {
                // DO [...] anonEventHandler
                fun = CreateAnonFunction(anonEventHandler);
            }
            else
            {
                // DO funName=Iden
                string funName = context.funName.GetText();
                if (!CurrentScope.Lookup(funName, out fun))
                {
                    throw Handler.MissingDeclaration(context.funName, "function", funName);
                }
            }

            // TODO: is this correct?
            fun.Role |= FunctionRole.EventHandler;

            // ON eventList
            List<IStateAction> actions = new List<IStateAction>();
            foreach (PParser.EventIdContext eventIdContext in context.eventList().eventId())
            {
                if (!CurrentScope.Lookup(eventIdContext.GetText(), out PEvent evt))
                {
                    throw Handler.MissingDeclaration(eventIdContext, "event", eventIdContext.GetText());
                }

                actions.Add(new EventDoAction(eventIdContext, evt, fun));
            }

            return actions.ToArray();
        }

        public override object VisitOnEventPushState(PParser.OnEventPushStateContext context)
        {
            // PUSH stateName
            State targetState = FindState(context.stateName());

            // ON eventList
            List<IStateAction> actions = new List<IStateAction>();
            foreach (PParser.EventIdContext token in context.eventList().eventId())
            {
                if (!CurrentScope.Lookup(token.GetText(), out PEvent evt))
                {
                    throw Handler.MissingDeclaration(token, "event", token.GetText());
                }

                actions.Add(new EventPushState(token, evt, targetState));
            }

            return actions.ToArray();
        }

        public override object VisitOnEventGotoState(PParser.OnEventGotoStateContext context)
        {
            Function transitionFunction;
            if (context.funName != null)
            {
                // WITH funName=Iden
                string funName = context.funName.GetText();
                if (!CurrentScope.Lookup(funName, out transitionFunction))
                {
                    throw Handler.MissingDeclaration(context.funName, "function", funName);
                }

                transitionFunction.Role |= FunctionRole.TransitionFunction;
            }
            else if (context.anonEventHandler() != null)
            {
                // WITH anonEventHandler
                transitionFunction = CreateAnonFunction(context.anonEventHandler());
                transitionFunction.Role |= FunctionRole.TransitionFunction;
            }
            else
            {
                // SEMI
                transitionFunction = null;
            }

            // GOTO stateName
            State target = FindState(context.stateName());

            // ON eventList
            List<IStateAction> actions = new List<IStateAction>();
            foreach (PParser.EventIdContext eventIdContext in context.eventList().eventId())
            {
                if (!CurrentScope.Lookup(eventIdContext.GetText(), out PEvent evt))
                {
                    throw Handler.MissingDeclaration(eventIdContext, "event", eventIdContext.GetText());
                }

                actions.Add(new EventGotoState(eventIdContext, evt, target, transitionFunction));
            }

            return actions.ToArray();
        }

        #endregion Machines

        #region Functions

        public override object VisitPFunDecl(PParser.PFunDeclContext context)
        {
            // FUN name=Iden
            Function fun = (Function)nodesToDeclarations.Get(context);

            // LPAREN funParamList? RPAREN
            Variable[] paramList = context.funParamList() != null
                ? (Variable[])Visit(context.funParamList())
                : new Variable[0];
            fun.Signature.Parameters.AddRange(paramList);

            // (COLON type)?
            fun.Signature.ReturnType = ResolveType(context.type());

            // functionBody
            // handled in later phase.
            return fun;
        }

        public override object VisitForeignFunDecl(PParser.ForeignFunDeclContext context)
        {
            // FUN name=Iden
            Function fun = (Function)nodesToDeclarations.Get(context);

            // LPAREN funParamList? RPAREN
            Variable[] paramList = context.funParamList() != null
                ? (Variable[])Visit(context.funParamList())
                : new Variable[0];
            fun.Signature.Parameters.AddRange(paramList);

            // (COLON type)?
            fun.Signature.ReturnType = ResolveType(context.type());

            // SEMI
            // no function body
            fun.Role |= FunctionRole.Foreign;

            // Creates
            foreach (PParser.IdenContext createdInterface in context._interfaces)
            {
                if (CurrentScope.Lookup(createdInterface.GetText(), out Interface @interface))
                {
                    fun.AddCreatesInterface(@interface);
                }
                else
                {
                    throw Handler.MissingDeclaration(createdInterface, "interface", createdInterface.GetText());
                }
            }
            return fun;
        }

        public override object VisitFunParamList(PParser.FunParamListContext context)
        {
            // funParamList : funParam (COMMA funParam)* ;
            return context.funParam().Select(Visit).Cast<Variable>().ToArray();
        }

        public override object VisitFunParam(PParser.FunParamContext context)
        {
            // funParam : name=iden COLON type ;
            Variable param = (Variable)nodesToDeclarations.Get(context);
            param.Type = ResolveType(context.type());
            return param;
        }

        #endregion Functions
    }
}